(function(root,factory){
  const api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  else root.MotorTuning=api;
})(globalThis,function(){
  'use strict';
  const number=(v,min,max,name)=>{
    if(typeof v!=='number'||!Number.isFinite(v)||v<min||v>max)throw new Error(`${name}范围为${min}到${max}`);
    return v;
  };
  const quantize=v=>Math.round(v*1000)/1000;
  function periodFromHz(hz,filter=false){
    number(hz,filter?50:20,1000,filter?'滤波更新频率':'速度环更新频率');
    return Math.round(1000/hz);
  }
  function normalize(value){
    if(!value||value.format!=='3motor-tuning'||![1,2,3].includes(value.version))throw new Error('不是受支持的3motor调参文件（format=3motor-tuning, version=1/2/3）');
    if(!Array.isArray(value.motors)||value.motors.length!==3)throw new Error('调参文件必须包含三台电机');
    const ids=value.motors.map(m=>m.id);
    if(new Set(ids).size!==3||ids.some(id=>![1,2,3].includes(id)))throw new Error('电机编号必须为1、2、3，不能重复');
    const motors=[...value.motors].sort((a,b)=>a.id-b.id).map(m=>({id:m.id,
      outputScale:quantize(number(m.outputScale,0,5,`M${m.id}输出比例`)),
      kp:quantize(number(m.kp,0,100000,'Kp')),ki:quantize(number(m.ki,0,100000,'Ki')),kd:quantize(number(m.kd,0,100000,'Kd')),
      pidDown: Object.fromEntries(['kp','ki','kd'].map(k => [k, quantize(number(value.version===1 ? m[k] : m.pidDown?.[k],0,100000,`下降${k}`))])),
      frictionRaw:number(m.frictionRaw,0,16384,'静摩擦幅值'),frictionMoveDeg:quantize(number(m.frictionMoveDeg,.001,1,'静摩擦撤销阈值'))
    }));
    if(motors.some(m=>!Number.isInteger(m.frictionRaw)))throw new Error('静摩擦幅值必须是raw整数');
    const targets=value.speedTargets??[0,0,0];
    if(!Array.isArray(targets)||targets.length!==3)throw new Error('需要三路目标速度');
    let timing=null;
    if(value.timing){
      const speedPeriodMs=number(value.timing.speedPeriodMs,1,50,'速度环周期ms');
      const filterPeriodMs=number(value.timing.filterPeriodMs,1,20,'滤波周期ms');
      if(!Number.isInteger(speedPeriodMs)||!Number.isInteger(filterPeriodMs))throw new Error('更新周期必须是整数毫秒');
      timing={speedPeriodMs,filterPeriodMs,cutoffHz:number(value.timing.cutoffHz??20,.1,24.9,'记录的截止频率Hz')};
    }
    let test=null;
    if(value.test){const t=value.test;if(typeof t.fixedGravity!=='boolean')throw new Error('固定模式补偿开关必须为布尔值');test={
      motor:number(t.motor,1,3,'测试电机'),limitA:number(t.limitA,.01,5,'增量上限A'),rampAps:number(t.rampAps,.01,2,'增量速率A/s'),thresholdDeg:number(t.thresholdDeg,.05,10,'位移阈值'),durationS:number(t.durationS,1,60,'测试时长'),
      fixedRaw:number(t.fixedRaw,-16384,16384,'固定电流'),fixedDurationS:number(t.fixedDurationS,1,60,'固定给定时长'),fixedGravity:Boolean(t.fixedGravity)};
      if(!Number.isInteger(test.motor)||!Number.isInteger(test.fixedRaw))throw new Error('电机编号和固定电流需为整数');
    }
    let position=null;
    if(value.position){
      const v=value.position;
      if(!Array.isArray(v.targets)||v.targets.length!==3||!Array.isArray(v.pid)||v.pid.length!==3)throw new Error('角度配置需要三路目标和PID');
      position={settling:v.settling?{toleranceDeg:quantize(number(v.settling.toleranceDeg,.001,10,'到位误差范围')),holdMs:number(v.settling.holdMs,1,5000,'到位稳定时间ms')}:null,targets:v.targets.map(a=>quantize(number(a,-180,180,'目标角度'))),pid:v.pid.map(p=>({
        up:Object.fromEntries(['kp','ki','kd'].map(k=>[k,quantize(number(p.up?.[k],0,100000,`角度上升${k}`))])),
        down:Object.fromEntries(['kp','ki','kd'].map(k=>[k,quantize(number(p.down?.[k],0,100000,`角度下降${k}`))])),
        maxSpeed:quantize(number(p.maxSpeed,.001,180,'角度环速度上限')),...(p.minSpeed==null?{}:{minSpeed:quantize(number(p.minSpeed,0,p.maxSpeed,'角度环最小速度'))})
      }))};
    }
    if(position?.settling&&!Number.isInteger(position.settling.holdMs))throw new Error('到位稳定时间必须为整数毫秒');
    return {format:'3motor-tuning',version:3,createdAt:value.createdAt??null,source:value.source??'file',kg:quantize(number(value.kg,0,5,'重力系数')),motors,
      speedTargets:targets.map(v=>quantize(number(v,-180,180,'目标速度'))),timing,test,position};
  }
  function defaults(){return normalize({format:'3motor-tuning',version:1,source:'draft',kg:1.4,
    motors:[1,2,3].map(id=>({id,outputScale:1,kp:0,ki:0,kd:0,frictionRaw:0,frictionMoveDeg:.05})),
    speedTargets:[0,0,0],timing:{speedPeriodMs:5,filterPeriodMs:5,cutoffHz:20}});}
  function fromFrame(frame){
    if(!frame?.pidSettings||!frame?.outputScale||!frame?.frictionMove)throw new Error('等待设备回传完整PID、比例和静摩擦参数');
    const c=normalize({format:'3motor-tuning',version:1,source:'device',kg:frame.kg,
      motors:frame.pidSettings.map((p,i)=>({id:i+1,outputScale:frame.outputScale[i],kp:p.kp,ki:p.ki,kd:p.kd,frictionRaw:p.friction,frictionMoveDeg:frame.frictionMove[i]})),
      speedTargets:frame.speedTarget||[0,0,0],timing:frame.timing||null});
    if(frame.pidDirectional) c.motors.forEach((m,i)=>{Object.assign(m,frame.pidDirectional[i].up);m.pidDown={...frame.pidDirectional[i].down};});
    if(frame.anglePid)c.position={targets:[...frame.angleTarget],pid:frame.anglePid.map(p=>({up:{...p.up},down:{...p.down},maxSpeed:p.maxSpeed,...(p.minSpeed==null?{}:{minSpeed:p.minSpeed})}))};
    if(frame.angleSettling&&c.position)c.position.settling={toleranceDeg:frame.angleSettling.toleranceDeg,holdMs:frame.angleSettling.holdMs};
    return normalize(c);
  }

  function commands(value){
    const c=normalize(value),lines=[`P,${Math.round(c.kg*1000)}`,`R,${c.motors.map(m=>Math.round(m.outputScale*1000)).join(',')}`];
    if(c.timing)lines.push(`U,${c.timing.speedPeriodMs},${c.timing.filterPeriodMs}`);
    for(const m of c.motors){
      for(const direction of [-1,1]){const p=direction<0?m:m.pidDown;lines.push(`K,${m.id},${direction},${Math.round(p.kp*1000)},${Math.round(p.ki*1000)},${Math.round(p.kd*1000)},${m.frictionRaw}`);}
      lines.push(`T,${m.id},${Math.round(m.frictionMoveDeg*1000)}`);
    }
    if(c.position)c.position.pid.forEach((p,i)=>{for(const direction of [-1,1]){
      const g=direction<0?p.up:p.down;lines.push(`J,${i+1},${direction},${Math.round(g.kp*1000)},${Math.round(g.ki*1000)},${Math.round(g.kd*1000)},${Math.round(p.maxSpeed*1000)}${p.minSpeed==null?'':','+Math.round(p.minSpeed*1000)}`);
    }});
    if(c.position?.settling)lines.push(`H,${Math.round(c.position.settling.toleranceDeg*1000)},${c.position.settling.holdMs}`);
    return lines;
  }
  function matchesFrame(config,frame){
    if(!frame?.pidSettings||!frame?.outputScale||!frame?.frictionMove)return false;
    const close=(a,b)=>Math.abs(a-b)<.00051;
    const gainsEqual=(a,b)=>['kp','ki','kd'].every(k=>close(a[k],b[k]));
    return close(config.kg,frame.kg)&&config.motors.every((m,i)=>{
      const p=frame.pidDirectional?.[i];
      return close(m.outputScale,frame.outputScale[i])&&gainsEqual(m,p?.up||frame.pidSettings[i])&&
        gainsEqual(m.pidDown,p?.down||frame.pidSettings[i])&&m.frictionRaw===frame.pidSettings[i].friction&&close(m.frictionMoveDeg,frame.frictionMove[i]);
    })&&(!config.position||frame.anglePid&&config.position.pid.every((p,i)=>
      gainsEqual(p.up,frame.anglePid[i].up)&&gainsEqual(p.down,frame.anglePid[i].down)&&close(p.maxSpeed,frame.anglePid[i].maxSpeed)&&(p.minSpeed==null||close(p.minSpeed,frame.anglePid[i].minSpeed))))&&(!config.position?.settling||frame.angleSettling&&close(config.position.settling.toleranceDeg,frame.angleSettling.toleranceDeg)&&config.position.settling.holdMs===frame.angleSettling.holdMs)&&(!config.timing||frame.timing&&frame.timing.speedPeriodMs===config.timing.speedPeriodMs&&frame.timing.filterPeriodMs===config.timing.filterPeriodMs);
  }

  function designFilter(cutoffHz,periodMs){
    const k=Math.tan(Math.PI*cutoffHz*periodMs/1000),norm=1/(1+Math.SQRT2*k+k*k);
    return {b0:k*k*norm,a1:2*(k*k-1)*norm,a2:(1-Math.SQRT2*k+k*k)*norm};
  }
  function resetFilter(c,value=0){return {s1:(1-c.b0)*value,s2:(c.b0-c.a2)*value};}
  function filterStep(state,c,input){const y=c.b0*input+state.s1;state.s1=2*c.b0*input-c.a1*y+state.s2;state.s2=c.b0*input-c.a2*y;return y;}
  function response(cutoffHz,periodMs,frequency){
    const fs=1000/periodMs;
    if(frequency<=0||frequency>=fs/2)return null;
    const r=Math.tan(Math.PI*frequency/fs)/Math.tan(Math.PI*cutoffHz/fs);
    const phase=-Math.atan2(Math.SQRT2*r,1-r*r)*180/Math.PI;
    return {frequency,phase,gainDb:-10*Math.log10(1+r**4),delayMs:-phase/(360*frequency)*1000};
  }
  return {defaults,normalize,fromFrame,commands,matchesFrame,periodFromHz,response,designFilter,resetFilter,filterStep};
});
