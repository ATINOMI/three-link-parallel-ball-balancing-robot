/* Local USB control only. No movement is sent before Start or the user's O/Home action. */
(()=>{
  'use strict';
  const $=id=>document.getElementById(id),K=RrsKinematics;
  let frame=null,hand=null,polling=false,feedbackAt=0,feedbackValue='',connecting=false;
  const control=new PlateControl.Control({send:line=>link.send(line)});
  const link=new MotorSerial.SerialLink({
    onFrame:f=>{frame=f;control.acceptFrame(f);render();},
    onStatus:()=>{if(!link.connected&&control.active)control.stop('平台 USB 已断开');render();},
    onError:error=>{control.stop('USB 错误：'+error.message);render();},
    onLegacy:()=>{control.reason='固件版本过旧，请烧录 V10';render();}
  });
  const mappingKey='3rrs-axis-mapping-v1';
  let mappingSaved=true,mappingNotice='';
  try {
    const stored=localStorage.getItem(mappingKey);
    if(stored)control.setMapping(JSON.parse(stored));
  }catch(error){control.mapping={...PlateControl.DEFAULT_MAPPING};mappingSaved=false;}
  const axisNames=['axis-preset','axis-angle','axis-swap','axis-pitch-sign','axis-roll-sign','axis-reset','pose-sensitivity'];
  function renderMapping(){
    const m=control.mapping,known=[150,0,30,-90].includes(m.rotationDeg);
    $('axis-preset').value=known?String(m.rotationDeg):'custom';
    $('pose-sensitivity').value=String(m.sensitivity);
    $('axis-angle').value=String(m.rotationDeg);$('axis-swap').checked=m.swap;
    $('axis-pitch-sign').value=String(m.pitchSign);$('axis-roll-sign').value=String(m.rollSign);
    const axes=PlateControl.mappedAxes(m),angle=v=>((Math.atan2(v[1],v[0])*K.DEG+360)%360).toFixed(0)+'°';
    $('axis-summary').textContent='Pitch + → '+angle(axes.pitch)+' · Roll + → '+angle(axes.roll);
  }
  function editMapping(change){
    if(control.active)return;
    try {
      control.setMapping({...control.mapping,...change});mappingNotice='';
      try{localStorage.setItem(mappingKey,JSON.stringify(control.mapping));mappingSaved=true;}
      catch(error){mappingSaved=false;}
      renderMapping();render();
    }catch(error){mappingNotice=error.message;renderMapping();render();}
  }
  $('axis-preset').onchange=()=>{if($('axis-preset').value!=='custom')editMapping({rotationDeg:Number($('axis-preset').value),swap:false});};
  $('axis-angle').onchange=()=>editMapping({rotationDeg:$('axis-angle').value.trim()===''?NaN:Number($('axis-angle').value)});
  $('axis-swap').onchange=()=>editMapping({swap:$('axis-swap').checked});
  $('axis-pitch-sign').onchange=()=>editMapping({pitchSign:Number($('axis-pitch-sign').value)});
  $('axis-roll-sign').onchange=()=>editMapping({rollSign:Number($('axis-roll-sign').value)});
  $('axis-reset').onclick=()=>editMapping({...PlateControl.DEFAULT_MAPPING,sensitivity:control.mapping.sensitivity});
  $('pose-sensitivity').onchange=()=>editMapping({sensitivity:$('pose-sensitivity').value.trim()===''?NaN:Number($('pose-sensitivity').value)});
  PlateScene.bindAlignment({isLocked:()=>control.active,onRotate:rotationDeg=>editMapping({rotationDeg})});
  renderMapping();
  function render(){
    const current=performance.now(),motorFresh=link.connected&&frame&&current-control.frameAt<500;
    $('hand-status').textContent=!hand?.connected?'手柄 · 等待 USB':hand.input_version!==2?'手柄 · 请重启新版服务':hand.calibrating?'手柄 · 校准 '+Math.round(hand.calibration*100)+'%':hand.ready?'手柄 · 已就绪':'手柄 · 等待 IMU';
    $('motor-status').textContent=link.connected?'平台 · '+(frame?.version||'等待遥测'):'平台 · 未连接';
    $('connect').textContent=link.connected?'断开平台 USB':connecting?'连接中…':'连接平台 USB';$('connect').disabled=connecting;
    const ready=motorFresh&&['V10','V11'].includes(frame.version)&&frame.fresh===7&&hand?.ready&&hand.input_version===2&&!hand.calibrating&&!hand.demo;
    $('axis-fields').disabled=control.active;
    axisNames.forEach(id=>$(id).disabled=control.active);
    $('axis-help').textContent=control.active?'控制中已锁定；按 Q 停止后可调整。':'拖动彩色正向箭头调整；平台方位从 +X 逆时针计算，两轴始终垂直。';
    $('axis-save-state').textContent=mappingNotice||(mappingSaved?'本机自动保存':'仅本次生效（浏览器未保存）');
    $('start').disabled=!ready||['running','homing'].includes(control.state);$('home').disabled=!ready||control.state==='homing';
    $('state-title').textContent=control.invalid?'姿态不可达':({idle:'未开始 / 已停止',running:'姿态跟随中',homing:'复位中',holding:'保持初始角度'})[control.state];
    $('message').textContent=control.reason;$('state-box').className='state '+(control.invalid?'invalid':control.active?'running':'');
    $('roll').textContent=motorFresh?(frame.pose[0]*K.DEG).toFixed(2):'—';
    $('pitch').textContent=motorFresh?(frame.pose[1]*K.DEG).toFixed(2):'—';
    $('height').textContent=motorFresh?(frame.pose[2]*1000).toFixed(2):'—';
    [1,2,3].forEach(i=>$('m'+i).textContent=motorFresh?frame.angles[i-1].toFixed(2):'—');
    $('visual-label').textContent=motorFresh?'设备反馈姿态（正解估计）':'初始姿态示意';
    $('telemetry').textContent=motorFresh?'模式 '+frame.mode+' · 电流 raw ['+frame.total.join(', ')+']':'等待设备数据';
  }
  $('connect').onclick=async()=>{
    if(link.connected){control.stop('平台已断开');await link.disconnect();frame=null;render();return;}
    connecting=true;render();try{await link.connect();control.reason='等待固件与手柄就绪';}catch(e){control.reason=e.message;}finally{connecting=false;render();}
  };
  $('start').onclick=()=>{try{control.start();}catch(e){control.reason=e.message;}render();};
  $('home').onclick=()=>{try{control.home();}catch(e){control.reason=e.message;}render();};
  const stop=reason=>{control.stop(reason);render();};
  $('stop').onclick=()=>stop('已急停 · 电流清零');
  window.addEventListener('keydown',e=>{if(e.code==='KeyQ'||e.key.toLowerCase()==='q'){e.preventDefault();stop('Q 键急停 · 电流清零');}},true);
  document.addEventListener('visibilitychange',()=>{
    control.inputEnabled=!document.hidden;
    if(document.hidden&&control.active)stop('控制页面已隐藏，电流清零');
    if(!document.hidden)control.handAt=-Infinity; // Consume old button presses without starting motion.
  });
  window.addEventListener('pagehide',()=>{control.stop('页面关闭');fetch('/api/feedback/idle',{method:'POST',keepalive:true}).catch(()=>{});});
  async function poll(){
    if(polling)return;polling=true;
    try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(250)});if(!response.ok)throw Error('手柄服务异常');
      hand=await response.json();control.acceptHand(hand);
    }catch(e){if(control.active)stop('手柄服务连接中断');hand=null;}
    finally{polling=false;render();}
  }
  setInterval(poll,33);poll();
  setInterval(()=>{
    control.tick();
    const now=performance.now();
    if(now-feedbackAt>100||feedbackValue!==control.feedback){feedbackAt=now;feedbackValue=control.feedback;
      fetch('/api/feedback/'+feedbackValue,{method:'POST',signal:AbortSignal.timeout(250)}).catch(()=>{});}
    render();
  },50);
  function draw(){const fresh=frame&&link.connected&&performance.now()-control.frameAt<500;
    PlateScene.draw(fresh?frame.pose:K.homePose,fresh?frame.angles:K.homeAngles,control.active?control.pose:null,fresh,control.mapping,$('axis-editor').open);
    requestAnimationFrame(draw);
  }draw();render();
})();
