'use strict';
const $=id=>document.getElementById(id);
let target=[1,0,0,0],displayed=[1,0,0,0],ready=false,lastFrame=performance.now(),latest=null;
const format=v=>`${Math.abs(v)<.05?'':v>0?'+':''}${(Math.abs(v)<.05?0:v).toFixed(1)}`;
const vector=v=>v?.map(n=>n.toFixed(4)).join('  /  ')||'—';
function show(data){
  latest=data;ready=Boolean(data.ready);if(ready)target=data.quaternion;
  $('connectionDot').classList.toggle('online',ready);
  $('connectionText').textContent=data.demo?'演示模式':data.connected?(ready?'USB 已连接':'等待传感器'):'等待 USB 手柄';
  $('modeText').textContent=data.demo?'演示 / 非手柄数据':'USB / 六轴姿态';
  $('waiting').hidden=ready;
  $('waiting').textContent=data.connected?'等待有效 IMU 数据…':'连接 USB 手柄，开始转动。';
  ['roll','pitch','yaw'].forEach((id,i)=>{$(id).textContent=ready?format(data.angles[i]):'—';});
  $('rateText').textContent=ready?(data.demo?'离线旋转演示':`${Math.round(data.hz)} Hz · ${Math.round(data.age_ms)} ms`):'等待传感器数据';
  $('device').textContent=data.device||'等待 USB DualSense';$('gyro').textContent=vector(data.gyro);$('accel').textContent=vector(data.accel);$('bias').textContent=vector(data.bias);
  $('zero').disabled=!ready;$('calibrate').disabled=!ready||data.demo;
  $('calibrationBar').style.width=`${Math.round((data.calibration||0)*100)}%`;
  $('hint').textContent=data.demo?'演示数据，实际使用请通过USB连接。':!data.connected?(data.message?.includes('USB')?'请使用 USB 连接 DualSense。':'等待手柄连接。'):!ready?'设备已连接，等待新的传感器数据。':data.calibrating?`请保持静止 2 秒 · ${Math.round(data.calibration*100)}%`:'零偏已校准。归零可把当前姿态设为参考。';
}
async function poll(){
  const start=performance.now();
  try{const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!response.ok)throw Error('http');const data=await response.json();if(data.app!=='dualforge-attitude')throw Error('application');show(data);}
  catch{ready=false;$('connectionDot').classList.remove('online');$('connectionText').textContent='服务未连接';$('hint').textContent='请启动姿态程序，页面会自动重新连接。';$('zero').disabled=$('calibrate').disabled=true;$('waiting').hidden=false;$('waiting').textContent='等待本地姿态服务…';}
  setTimeout(poll,Math.max(8,33-(performance.now()-start)));
}
async function command(name){
  if(!latest?.ready)return;
  try{const response=await fetch(`/api/${name}`,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});if(!response.ok)throw Error('request');}
  catch{$('hint').textContent='操作未送达，请检查姿态服务。';}
}
$('zero').onclick=()=>command('zero');$('calibrate').onclick=()=>command('calibrate');
function frame(now){const dt=Math.min(.1,(now-lastFrame)/1000);lastFrame=now;displayed=SketchCube.interpolate(displayed,target,1-Math.exp(-dt/.025));SketchCube.draw($('cube'),displayed,ready);requestAnimationFrame(frame);}
requestAnimationFrame(frame);poll();
