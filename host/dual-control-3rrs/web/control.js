(function (root, factory) {
  const api=factory(typeof module==='object'&&module.exports?require('./kinematics.js'):root.RrsKinematics);
  if(typeof module==='object'&&module.exports)module.exports=api;else root.PlateControl=api;
})(globalThis,function(K){
  'use strict';
  const wrap = x => ((x+180)%360+360)%360-180;
  const HEIGHT_STEP_DEG=5;
  const M3_AZIMUTH=4*Math.PI/3;
  const DEFAULT_MAPPING=Object.freeze({rotationDeg:150,swap:false,rollSign:1,pitchSign:1,sensitivity:1});
  function normalizeMapping(value){
    if(!value||!Number.isFinite(value.rotationDeg)||value.rotationDeg < -180||value.rotationDeg > 180||
      typeof value.swap!=='boolean'||![-1,1].includes(value.rollSign)||![-1,1].includes(value.pitchSign))
      throw Error('轴向角度应为 −180°～180°，方向应为正向或反向');
    // Old saved mappings have no sensitivity: extend them without changing any axis setting.
    const sensitivity=value.sensitivity===undefined?1:value.sensitivity;
    if(!Number.isFinite(sensitivity)||sensitivity<0||sensitivity>10)
      throw Error('姿态灵敏度应为 0～10 倍');
    return {rotationDeg:value.rotationDeg,swap:value.swap,rollSign:value.rollSign,pitchSign:value.pitchSign,sensitivity};
  }
  function mappedAxes(mapping=DEFAULT_MAPPING){
    const roll=(mapping.rotationDeg+(mapping.swap?90:0))*K.RAD;
    const pitch=(mapping.rotationDeg+(mapping.swap?0:90))*K.RAD;
    return {roll:[Math.cos(roll)*mapping.rollSign,Math.sin(roll)*mapping.rollSign],
      pitch:[Math.cos(pitch)*mapping.pitchSign,Math.sin(pitch)*mapping.pitchSign]};
  }
  function rotationForAxis(axis,azimuthDeg,mapping){
    const offset=axis==='pitch'?(mapping.swap?0:90):(mapping.swap?90:0);
    const sign=axis==='pitch'?mapping.pitchSign:mapping.rollSign;
    return wrap(azimuthDeg-offset-(sign<0?180:0));
  }
  // Rotate/swap/sign the input frame; the default pitch axis points radially toward M3.
  // Apply finite rotations to the base normal, then recover the model's psi_x / psi_y.
  // Dependent yaw is still computed exclusively by the supplied strict model.
  function mapTilt(base,roll,pitch,mapping=DEFAULT_MAPPING){
    const sensitivity=mapping.sensitivity??1;
    roll*=mapping.rollSign*sensitivity;pitch*=mapping.pitchSign*sensitivity;
    if(mapping.swap)[roll,pitch]=[pitch,roll];
    const azimuth=mapping.rotationDeg*K.RAD,c=Math.cos(azimuth),s=Math.sin(azimuth);
    const nx=Math.sin(base[1]),ny=-Math.sin(base[0])*Math.cos(base[1]),nz=Math.cos(base[0])*Math.cos(base[1]);
    const x=c*nx+s*ny,y=-s*nx+c*ny;
    const xp=Math.cos(pitch)*x+Math.sin(pitch)*nz,zp=-Math.sin(pitch)*x+Math.cos(pitch)*nz;
    const yr=Math.cos(roll)*y-Math.sin(roll)*zp,zr=Math.sin(roll)*y+Math.cos(roll)*zp;
    const X=c*xp-s*yr,Y=s*xp+c*yr;
    return [Math.atan2(-Y,zr),Math.atan2(X,Math.hypot(Y,zr))];
  }
  // A height's equivalent angle is the common actuator angle of the LEVEL platform.
  // At nonzero tilt, only Z changes; individual joint increments need not all be 5 degrees.
  function stepHeight(z,direction){
    const flat=K.inverse([0,0,z]);
    if(!flat.ok)return flat;
    const theta=flat.angles[0]-direction*HEIGHT_STEP_DEG,t=theta*K.RAD;
    const {r0,rp,l1,l2}=K.geometry;
    const square=l2*l2-(rp-r0-l1*Math.cos(t))**2;
    const nextZ=-l1*Math.sin(t)+Math.sqrt(square);
    const check=K.inverse([0,0,nextZ]);
    if(!check.ok||direction*(nextZ-z)<=0||Math.abs(check.angles[0]-theta)>1e-6)
      return {ok:false,reason:'该 5° 等效高度步进超出可达分支'};
    return {ok:true,z:nextZ};
  }
  class Control {
    constructor({send, now=()=>performance.now()}) {
      this.send=send;this.now=now;this.mapping={...DEFAULT_MAPPING};this.state='idle';this.reason='连接设备后开始';
      this.pose=[...K.homePose];this.targets=[...K.homeAngles];this.invalid=false;
      this.frameAt=this.handAt=-Infinity;this.circle=null;this.hand=null;this.frame=null;
      this.up=this.down=null;this.heightSteps=0;this.heightRequest=K.homeZ;this.heightError=null;
      this.session=null;this.armedAt=0;this.confirmed=false;this.feedback='idle';this.inputEnabled=true;
    }
    setMapping(mapping){
      if(this.active)throw Error('先停止控制，再调整轴向');
      this.mapping=normalizeMapping(mapping);
    }
    get active(){return ['running','homing','holding'].includes(this.state);}
    acceptFrame(frame){
      this.frame=frame;this.frameAt=this.now();
      if(!this.active)return;
      if(!['V10','V11'].includes(frame.version)||frame.fresh!==7){this.stop('电机反馈中断或固件不匹配');return;}
      if(frame.mode===6)this.confirmed=true;
      if(this.confirmed&&frame.mode!==6){this.stop('固件已退出角度控制');return;}
      if(this.state==='homing'&&frame.mode===6&&
        frame.angleTarget.every((a,i)=>Math.abs(a-K.homeAngles[i])<.002)&&
        frame.angles.every((a,i)=>Math.abs(a-K.homeAngles[i])<=frame.angleSettling.toleranceDeg)&&
        frame.angleSettling.state.every(s=>s===2)){
        this.pose=[...K.homePose];this.reference=[...this.hand.angles];this.base=[0,0];
        this.state=this.resume?'running':'holding';this.reason=this.resume?'跟随中':'已复位 · 保持初始角度';
      }
    }
    acceptHand(hand){
      const buttonsEnabled=this.inputEnabled&&this.now()-this.handAt<250&&this.session===hand.session;
      if(this.session!==null&&this.session!==hand.session&&this.active)this.stop('手柄重新连接，请重新开始');
      this.session=hand.session;this.hand=hand;this.handAt=this.now();
      const previous=this.circle,previousUp=this.up,previousDown=this.down;
      this.circle=hand.circle_seq;this.up=hand.dpad_up_seq;this.down=hand.dpad_down_seq;
      if(buttonsEnabled&&previous!==null&&previous!==hand.circle_seq&&hand.ready&&!hand.demo){
        try{this.home();}catch(error){this.reason=error.message;}
        return;
      }
      if(buttonsEnabled&&this.state==='running'&&hand.ready&&!hand.demo&&previousUp!=null&&previousDown!=null)
        this.heightSteps+=(this.up-previousUp)-(this.down-previousDown);
    }
    ready(){
      if(this.hand?.input_version!==2)throw Error('请关闭旧服务并重新启动新版上位机');
      if(!this.frame||this.now()-this.frameAt>500||this.frame.fresh!==7)throw Error('等待三路新鲜电机反馈');
      if(!['V10','V11'].includes(this.frame.version))throw Error('请先烧录本项目 V10 / V11 固件');
      if(!this.hand||!this.hand.ready||this.hand.calibrating||this.hand.demo||this.now()-this.handAt>250)
        throw Error('等待 USB 手柄静止校准完成');
    }
    arm(){this.heightSteps=0;this.heightError=null;this.armedAt=this.now();this.confirmed=false;this.lastTick=this.now();this.invalid=false;this.feedback='normal';}
    start(){
      this.ready();
      const pose=[...this.frame.pose], solution=K.inverse(pose,this.frame.angles);
      if(!solution.ok)throw Error('当前平台姿态不可达，先按 O 回到初始角度');
      if(solution.angles.some((v,i)=>Math.abs(v-this.frame.angles[i])>2))
        throw Error('当前反馈与初始装配分支不一致，请确认上电标定位置');
      this.pose=pose;this.heightRequest=pose[2];this.targets=solution.angles;this.base=pose.slice(0,2);
      this.reference=[...this.hand.angles];this.state='running';this.reason='跟随中';this.arm();this.output();
    }
    home(){
      this.ready();this.resume=this.state==='running';this.state='homing';this.reason='回到初始角度';
      this.pose=[...K.homePose];this.heightRequest=K.homeZ;this.targets=[...K.homeAngles];this.arm();this.output();
    }
    output(){this.send('A,'+this.targets.map(a=>Math.round(a*1000)).join(','));}
    tick(){
      if(!this.active){this.lastTick=this.now();return;}
      const now=this.now(),dt=(now-this.lastTick)/1000;this.lastTick=now;
      if(now-this.frameAt>500||now-this.handAt>250||!this.hand.ready||this.hand.calibrating){this.stop('设备数据超时或手柄未就绪');return;}
      if(dt>.2){this.stop('控制页面暂停，请重新开始');return;}
      if(!this.confirmed&&now-this.armedAt>1000){this.stop('固件未确认角度控制');return;}
      if(this.state==='running'){
        if(this.heightSteps){
          let height=this.pose[2];this.heightError=null;
          const direction=Math.sign(this.heightSteps);
          for(let n=0;n<Math.abs(this.heightSteps);n++){
            const step=stepHeight(height,direction);
            if(!step.ok){this.heightError=step;break;}
            height=step.z;
          }
          this.heightRequest=height;this.heightSteps=0;
        }
        const tilt=mapTilt(this.base,wrap(this.hand.angles[0]-this.reference[0])*K.RAD,
          wrap(this.hand.angles[1]-this.reference[1])*K.RAD,this.mapping);
        const candidate=[...tilt,this.heightRequest];
        this.request=candidate;
        const solution=this.heightError||K.reachableStep(this.pose,candidate,this.targets);
        this.invalid=!solution.ok;
        if(solution.ok){this.pose=candidate;this.targets=solution.angles;this.reason='跟随中';}
        else this.reason=solution.reason+' · 保持上一有效目标';
      }
      this.feedback=this.invalid?'invalid':'normal';this.output();
    }
    stop(reason='已急停 · 电流清零'){
      this.state='idle';this.heightSteps=0;this.invalid=false;this.feedback='idle';this.reason=reason;this.send('S');
    }
  }
  return {Control,mapTilt,stepHeight,HEIGHT_STEP_DEG,M3_AZIMUTH,DEFAULT_MAPPING,normalizeMapping,mappedAxes,rotationForAxis};
});
