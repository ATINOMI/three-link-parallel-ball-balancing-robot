'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const Core = require('../core.js');
const {SerialLink} = require('../serial.js');
const Model = require('../drawing.js');

function makeFrame(overrides={}) {
  return {ms:1,version:'V4',mode:1,motor:0,fresh:7,kg:1.4,outputScale:[1,1,1],
    angles:[12.77,12.77,12.77],pose:[0,0,.077],base:[-300,-310,-320],delta:[0,0,0],
    total:[-300,-310,-320],feedback:[0,0,0],speedMeasured:[0,0,0],speedTarget:[0,0,0],
    pidSettings:[0,1,2].map(()=>({kp:0,ki:0,kd:0,friction:0})),pidOutput:[0,0,0],frictionOutput:[0,0,0],...overrides};
}

test('actual firmware V4 packet decodes all 52 fields and split USB frames',()=>{
  const wire=fs.readFileSync(path.join(__dirname,'fixtures/speed-v4.txt'),'utf8');
  assert.equal(wire.trim().split(',').length,52);
  const f=Core.parseTelemetry(wire);
  assert.equal(f.version,'V4');assert.equal(f.mode,5);
  assert.deepEqual(f.speedTarget,[10,-10,0]);assert.deepEqual(f.speedMeasured,[0,0,0]);
  assert.deepEqual(f.pidSettings[0],{kp:10,ki:0,kd:0,friction:100});
  assert.deepEqual(f.pidOutput,[100,0,0]);assert.deepEqual(f.frictionOutput,[100,0,0]);
  assert.deepEqual(f.total,[-100,-310,-320]);
  const frames=[],reader=new Core.LineReader(s=>{const v=Core.parseTelemetry(s);if(v)frames.push(v);});
  for(let i=0;i<wire.length;i+=13)reader.push(wire.slice(i,i+13));
  assert.equal(frames.length,1);assert.equal(Core.parseTelemetry(wire.replace('V4','V3')),null);
});

test('speed target confirmation, heartbeat, update, timeout, emergency and legacy rejection',()=>{
  let now=0;const commands=[],results=[];
  const loop=new Core.SpeedLoop({send:s=>commands.push(s),onFinish:s=>results.push(s),now:()=>now});
  loop.accept(makeFrame({speedMeasured:null}));assert.throws(()=>loop.start([1,0,0]),/V4/);
  loop.accept(makeFrame());loop.start([-3,4,0]);assert.equal(commands.at(-1),'V,-3000,4000,0');
  loop.accept(makeFrame({mode:5,speedTarget:[-2,4,0]}));assert.equal(loop.run.phase,'arming');
  loop.accept(makeFrame({mode:5,speedTarget:[-3,4,0]}));assert.equal(loop.run.phase,'tracking');
  now=50;loop.tick();assert.equal(commands.at(-1),'V,-3000,4000,0');
  loop.start([0,0,0]);assert.equal(commands.at(-1),'V,0,0,0');
  loop.accept(makeFrame({mode:5}));loop.finish();assert.equal(commands.at(-1),'S');
  loop.accept(makeFrame());loop.start([1,0,0]);const count=commands.length;
  loop.accept(makeFrame({mode:0}));assert.equal(commands.length,count);assert.equal(loop.run,null);
  loop.accept(makeFrame());loop.start([1,0,0]);now+=501;loop.tick();assert.equal(commands.at(-1),'S');
  loop.accept(makeFrame());assert.throws(()=>loop.start([181,0,0]));
  loop.start([1,0,0]);now+=1501;loop.accept(makeFrame());loop.tick();assert.equal(commands.at(-1),'S');
});

test('queued speed targets are invalidated by stop; disconnect after stop never sends G',async()=>{
  const writes=[];const link=new SerialLink({onFrame(){},onStatus(){},onError(e){throw e;},onLegacy(){}});
  link.connected=true;link.writer={async write(b){writes.push(new TextDecoder().decode(b).trim());},releaseLock(){}};
  link.port={async close(){}};
  link.send('V,-1000,0,0');link.send('V,-2000,0,0');link.send('B,1');await link.send('S');
  await link.disconnect();assert.deepEqual(writes,['S']);
});

function appHarness(withWindows = false) {
  const makeClasses = () => {
    const classes = new Set();
    return {contains:key=>classes.has(key),add:(...keys)=>keys.forEach(key=>classes.add(key)),remove:(...keys)=>keys.forEach(key=>classes.delete(key)),
      toggle(key,force){const on=force===undefined?!classes.has(key):force;if(on)classes.add(key);else classes.delete(key);return on;}};
  };
  let time=0;const intervals=[],nodes=new Map(),windowEvents={},documentEvents={},physicalWrites=[],downloads=[],blobs=new Map();
  const values={motorSelect:'1',positionMotorSelect:'1',positionDirectionSelect:'-1',pidMotorSelect:'1',kgInput:'1.4',limitInput:'.5',rampInput:'.1',thresholdInput:'.5',durationInput:'15',fixedRawInput:'0',fixedDurationInput:'10'};
  const node=id=>{
    if(!nodes.has(id))nodes.set(id,{id,value:values[id]||'0',textContent:'',innerHTML:'',disabled:false,
      hidden:id==='positionParamsWindow'||id==='positionTrackWindow'||id==='speedParamsWindow'||id==='speedTrackWindow',style:{},dataset:{},events:{},
      classList:makeClasses(),setAttribute(){},removeAttribute(){},focus(){},click(){if(this.onclick)this.onclick();},closest(){return id.includes('Btn')?this:null;},
      addEventListener(type,fn){(this.events[type]||=[]).push(fn);},
      getBoundingClientRect(){return {left:parseFloat(this.style.left)||100,top:parseFloat(this.style.top)||80,width:430,height:id.includes('Handle')?70:650};}});
    return nodes.get(id);
  };
  const cards=[0,1,2].map(i=>({...node('card'+i),dataset:{motor:String(i)}}));
  const sandbox={Blob,URL:{createObjectURL(blob){const url='blob:test-'+blobs.size;blobs.set(url,blob);return url;},revokeObjectURL(url){blobs.delete(url);}},performance:{now:()=>time},setInterval(fn){intervals.push(fn);},console,
    document:{getElementById:node,documentElement:{style:{setProperty(){}}},querySelector:()=>({getBoundingClientRect:()=>({bottom:withWindows?58:0})}),createElement(){return {click(){downloads.push({name:this.download,blob:blobs.get(this.href)});}};},querySelectorAll:s=>s==='.motor-card'?cards:[],hidden:false,addEventListener(t,f){(documentEvents[t]||=[]).push(f);}},
    window:{innerWidth:1200,innerHeight:900,addEventListener(t,f){(windowEvents[t]||=[]).push(f);}},
    MotorDrawing:{...Model,mechanism(){},trend(){}},MotorSerial:{SerialLink:class{
      constructor(){this.connected=false;this.failed=false;this.emergencyLatched=false;}
      send(s){physicalWrites.push(s);return Promise.resolve(true);}
    }}};
  vm.createContext(sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../core.js'),'utf8'),sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../tuning.js'), 'utf8'), sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../app.js'),'utf8'),sandbox);
  if(withWindows) vm.runInContext(fs.readFileSync(path.join(__dirname,'../windows.js'),'utf8'),sandbox);
  const value=s=>vm.runInContext(s,sandbox);
  const advance=(n=1)=>{for(let i=0;i<n;i++){time+=50;intervals.forEach(f=>f());}};
  const event=(where,type,e)=>{for(const fn of (where==='document'?documentEvents[type]:where==='window'?windowEvents[type]:node(where).events[type])||[])fn({preventDefault(){},stopPropagation(){},...e});};
  return {node,value,advance,event,physicalWrites,downloads};
}

test('actual app: independent tuning, measured speed converges, target reversal, graph close, Q stop',()=>{
  const h=appHarness();h.node('demoBtn').onclick();
  assert.equal(h.value('ui.latest.version'),'V11');
  assert.equal(h.value('ui.latest.pidSettings.every(p=>p.kp===0 && p.ki===0 && p.kd===0 && p.friction===0)'),true);
  h.node('speedParamsOpenBtn').onclick();assert.equal(h.node('speedParamsWindow').hidden,false);
  for(let motor=1;motor<=3;motor++){
    h.node('pidMotorSelect').value=String(motor);h.node('pidMotorSelect').onchange();
    for (const direction of [-1,1]) {
      h.node('pidDirectionSelect').value=String(direction);h.node('pidDirectionSelect').onchange();
      h.node('pidKp').value='100';h.node('pidKi').value='100';h.node('pidKd').value='0';h.node('pidFriction').value='65';
      h.node('pidApplyBtn').onclick();h.advance();
    }
  }
  assert.equal(h.value('ui.pendingPid'),null);
  assert.equal(h.value('ui.latest.mode'),1); // applying parameters never starts motors
  h.node('speedTarget1').value='-3';h.node('speedTarget2').value='3';h.node('speedTarget3').value='0';
  h.node('speedStartBtn').onclick();h.advance(160);
  assert.equal(h.value('speedTest.run.phase'),'tracking');assert.equal(h.value('ui.latest.mode'),5);
  assert.ok(h.value('Math.abs(ui.latest.speedMeasured[0]+3)')<.1);
  assert.ok(h.value('Math.abs(ui.latest.speedMeasured[1]-3)')<.1);
  assert.ok(h.value('ui.latest.base[0]')<0);
  assert.equal(h.value('ui.latest.base[1]'),0); // descending motor does not receive gravity
  assert.equal(h.value('ui.latest.frictionOutput[0]'),0);assert.equal(h.value('ui.latest.frictionOutput[1]'),0);
  assert.equal(h.value('ui.latest.frictionOutput[2]'),0);
  assert.equal(h.node('upBtn').disabled,true);assert.equal(h.node('fixedApplyBtn').disabled,true);
  assert.ok(h.value('ui.history.filter(f=>f.mode===5 && f.speedMeasured).length')>100);
  h.node('speedTimeWindow').value='2';h.node('speedPauseBtn').onclick();
  const frozenEnd=h.value('ui.charts.speed.snapshot.at(-1).t');
  h.advance(4);
  assert.equal(h.value('ui.charts.speed.snapshot.at(-1).t'),frozenEnd);
  assert.ok(h.value('ui.history.at(-1).t')>frozenEnd);
  assert.equal(h.value('speedTest.run.phase'),'tracking');
  h.node('speedOffset').value='1';h.node('speedOffset').oninput();
  assert.equal(h.value('plotView("speed").options.end'),frozenEnd-1000);
  h.node('responsePauseBtn').onclick();h.advance();assert.ok(h.value('speedTest.run'));
  h.node('responsePauseBtn').onclick();h.node('speedPauseBtn').onclick();
  assert.equal(h.value('ui.charts.speed.paused'),false);
  assert.equal(h.value('ui.charts.speed.snapshot'),null);

  h.node('pidMotorSelect').value='1';h.node('pidMotorSelect').onchange();h.advance();
  assert.equal(h.node('frictionRestoreBtn').disabled,false);
  h.node('frictionMoveInput').value='0.08';h.node('frictionMoveApplyBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.frictionMove[0]'),.08);
  assert.equal(h.value('ui.latest.frictionMove[1]'),.05);
  assert.match(h.node('frictionMoveState').textContent,/0.080/);
  const integralBefore=h.value('ui.demo.pidState[0].integral');
  h.node('pidFriction').value='999'; // unsaved value must not be submitted by restore
  h.node('frictionRestoreBtn').onclick();
  assert.equal(h.value('ui.demo.frictionGate[0].starting'),true);
  assert.equal(h.value('ui.demo.pidSettings[0].friction'),65);
  assert.equal(h.value('ui.demo.pidState[0].integral'),integralBefore);
  assert.equal(h.value('ui.demo.targets[0]'),-3);
  h.advance();assert.equal(h.value('ui.latest.frictionOutput[0]'),0);
  assert.equal(h.value('ui.demo.frictionGate[0].starting'),false);
  h.node('speedTrackCloseBtn').onclick();assert.ok(h.value('speedTest.run'));
  h.node('speedTarget1').value='3';h.node('speedStartBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.frictionOutput[0]'),0); // moving reversal cannot re-arm static friction
  h.event('window','keydown',{code:'KeyQ',key:'q',repeat:false,target:h.node('pidKp')});h.advance(2);
  assert.equal(h.value('speedTest.run'),null);assert.equal(h.value('ui.latest.mode'),0);
  assert.equal(h.value('ui.latest.total.every(v=>v===0)'),true);assert.deepEqual(h.physicalWrites,[]);
});

test('new floating windows drag beyond title; tracking close leaves control running; parameter close stops',()=>{
  const h=appHarness();h.node('demoBtn').onclick();h.node('speedParamsOpenBtn').onclick();
  const title=h.node('speedParamsHandle');
  h.event('speedParamsHandle','mousedown',{button:0,clientX:130,clientY:100,target:title});
  h.event('document','mousemove',{clientX:330,clientY:220,target:h.node('pidKp')});
  assert.equal(h.node('speedParamsWindow').style.left,'300px');assert.equal(h.node('speedParamsWindow').style.top,'200px');
  h.event('document','mouseup',{});
  h.node('speedStartBtn').onclick();h.advance();assert.ok(h.value('speedTest.run'));
  h.node('speedParamsCloseBtn').onclick();h.advance();assert.equal(h.value('speedTest.run'),null);assert.equal(h.value('ui.latest.mode'),0);
});

test('static friction threshold ignores jitter, sums slow net displacement and clears at 22 counts',()=>{
  const gate=new Core.StaticFrictionGate();
  assert.equal(gate.update(0,-3,100),-1);
  for(const encoder of [1,-1,10,-10,0,10,0])assert.equal(gate.update(encoder,-3,101),-1);
  assert.equal(gate.update(21,-3,102),-1);assert.equal(gate.update(22,-3,103),0);
  assert.equal(gate.update(22,-9,500),0);
  gate.restore(22,-3);assert.equal(gate.update(43,-3,501),-1);assert.equal(gate.update(44,-3,502),0);
  gate.restore(44,3);assert.equal(gate.update(23,3,503),1);assert.equal(gate.update(22,3,504),0);
  assert.equal(gate.update(22,-3,505),0); // reversal during meaningful motion does not re-arm
  assert.equal(gate.update(22,0,600),0);assert.equal(gate.update(22,-3,601),-1);
  gate.thresholdDeg=.1;gate.restore(22,-3);assert.equal(gate.update(65,-3,602),-1);assert.equal(gate.update(66,-3,603),0);
  gate.thresholdDeg=.2;assert.equal(gate.update(66,-3,604),0); // threshold edits do not re-arm
});

test('manual restore records a new fixed origin; sub-threshold motion does not withdraw',()=>{
  const gate=new Core.StaticFrictionGate();gate.restore(100,-3);
  assert.equal(gate.update(121,-3,100),-1);assert.equal(gate.update(122,-3,101),0);
  gate.restore(122,3);assert.equal(gate.update(101,3,102),1);assert.equal(gate.update(100,3,103),0);
  gate.restore(100,0);assert.equal(gate.update(100,0,104),0);
});

test('manual restore never clears the local emergency latch or resumes gravity on disconnect',async()=>{
  const writes=[];const link=new SerialLink({onFrame(){},onStatus(){},onError(e){throw e;},onLegacy(){}});
  link.connected=true;link.emergencyLatched=true;
  link.writer={async write(b){writes.push(new TextDecoder().decode(b).trim());},releaseLock(){}};
  link.port={async close(){}};
  await link.send('B,1');assert.equal(link.emergencyLatched,true);
  await link.disconnect();assert.deepEqual(writes,['B,1']);
});

test('actual V5 firmware telemetry returns three independent displacement thresholds',()=>{
  const wire=fs.readFileSync(path.join(__dirname,'fixtures/friction-threshold-v5.txt'),'utf8');
  const frame=Core.parseTelemetry(wire);
  assert.equal(wire.trim().split(',').length,55);assert.equal(frame.version,'V5');
  assert.deepEqual(frame.frictionMove,[.08,.05,.07]);assert.equal(frame.mode,0);
  assert.equal(Core.parseTelemetry(wire+',1'),null);
});

test('tuning UI exports applied or unsaved three-motor drafts, imports without motion, applies config only and confirms rates',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();
  for(let id=1;id<=3;id++){
    h.node('pidMotorSelect').value=String(id);h.node('pidMotorSelect').onchange();
    h.node('pidKp').value=String(id*10);h.node('pidKi').value=String(id/10);h.node('pidKd').value='0.002';
    h.node('pidFriction').value=String(100+id);h.node('frictionMoveInput').value=String(id/10);
  }
  h.node('kgInput').value='1.25';h.node('outputScale1').value='.8';h.node('outputScale2').value='1.2';h.node('outputScale3').value='1.5';
  h.node('speedRateInput').value='500';h.node('filterRateInput').value='100';
  h.node('speedTarget1').value='-3';h.node('speedTarget2').value='2';h.node('speedTarget3').value='0';
  const profile=JSON.parse(JSON.stringify(h.value('buildTuningExport("draft")')));
  assert.deepEqual(profile.motors.map(m=>m.kp),[10,20,30]);assert.deepEqual(profile.speedTargets,[-3,2,0]);
  assert.deepEqual(profile.timing,{speedPeriodMs:2,filterPeriodMs:10,cutoffHz:20});
  const applied=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));
  assert.deepEqual(applied.motors.map(m=>m.kp),[0,0,0]);assert.equal(applied.kg,1.4);
  h.node('tuningExportSource').value='draft';h.node('tuningExportBtn').onclick();
  assert.equal(h.downloads.length,1);assert.match(h.downloads[0].name,/3motor-tuning-.*json/);
  const downloaded=JSON.parse(await h.downloads[0].blob.text());assert.deepEqual(downloaded.motors,profile.motors);
  h.node('tuningFileInput').files=[{text:async()=>JSON.stringify(profile)}];await h.node('tuningFileInput').onchange();h.advance();
  assert.equal(h.value('ui.latest.mode'),1);assert.equal(h.value('ui.demo.targets.every(v=>v===0)'),true);
  assert.equal(h.value('ui.latest.pidSettings.every(p=>p.kp===0)'),true);
  assert.equal(Number(h.node('pidKp').value),10);assert.equal(Number(h.node('speedTarget1').value),-3);
  await h.node('tuningApplyAllBtn').onclick();h.advance();
  assert.equal(h.value('ui.pendingTuning'),null);assert.equal(h.value('ui.latest.mode'),1);
  assert.equal(h.value('ui.demo.targets.every(v=>v===0)'),true);assert.equal(h.value('ui.latest.pidSettings[2].kp'),30);
  assert.equal(h.value('ui.latest.timing.speedPeriodMs'),2);assert.equal(h.value('ui.latest.timing.filterPeriodMs'),10);
  assert.match(h.node('tuningFileState').textContent,/回读确认/);
  h.node('speedRateInput').value='333';h.node('filterRateInput').value='200';h.node('timingApplyBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.timing.speedPeriodMs'),3);assert.match(h.node('timingState').textContent,/333.33Hz/);
  const before=h.value('JSON.stringify(ui.tuningDraft)');h.node('tuningFileInput').files=[{text:async()=>'{"format":"wrong"}'}];
  await h.node('tuningFileInput').onchange();assert.equal(h.value('JSON.stringify(ui.tuningDraft)'),before);
  assert.deepEqual(h.physicalWrites,[]);
});


test('directional PID UI keeps six drafts, applies only chosen bank, confirms independent of actual movement and switches demo PID on target sign',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();
  for(let motor=1;motor<=3;motor++)for(const direction of [-1,1]){
    h.node('pidMotorSelect').value=String(motor);h.node('pidMotorSelect').onchange();
    h.node('pidDirectionSelect').value=String(direction);h.node('pidDirectionSelect').onchange();
    h.node('pidKp').value=String(motor*100+(direction>0?10:0));h.node('pidKi').value=String(direction<0?1:2);h.node('pidKd').value='.003';
  }
  const draft=JSON.parse(JSON.stringify(h.value('buildTuningExport("draft")')));
  assert.equal(draft.version,3);assert.deepEqual(draft.motors.map(m=>m.kp),[100,200,300]);
  assert.deepEqual(draft.motors.map(m=>m.pidDown.kp),[110,210,310]);
  h.value('loadTuningFile('+JSON.stringify(draft)+')');await h.node('tuningApplyAllBtn').onclick();h.advance();
  assert.equal(h.value('ui.pendingTuning'),null);assert.equal(h.value('ui.latest.mode'),1);
  assert.deepEqual(JSON.parse(h.value('JSON.stringify(ui.latest.pidDirectional.map(p=>p.down.kp))')),[110,210,310]);
  h.node('pidDirectionSelect').value='1';h.node('pidDirectionSelect').onchange();
  assert.equal(Number(h.node('pidKp').value),110);h.node('pidKp').value='777';h.node('pidApplyBtn').onclick();h.advance();
  assert.equal(h.value('ui.pendingPid'),null);assert.equal(h.value('ui.latest.pidDirectional[0].up.kp'),100);
  assert.equal(h.value('ui.latest.pidDirectional[0].down.kp'),777);assert.match(h.node('pidParamState').textContent,/下降 Kp 777/);
  h.node('speedTarget1').value='2';h.node('speedStartBtn').onclick();h.advance();
  assert.equal(h.value('ui.demo.pidDirection[0]'),1);assert.equal(h.value('ui.latest.pidSettings[0].kp'),777);
  h.node('speedTarget1').value='-2';h.node('speedStartBtn').onclick();h.advance();
  assert.equal(h.value('ui.demo.pidDirection[0]'),-1);assert.equal(h.value('ui.latest.pidSettings[0].kp'),100);
  h.node('speedTarget1').value='0';h.node('speedStartBtn').onclick();h.advance();
  assert.equal(h.value('ui.demo.pidDirection[0]'),-1);
  const exported=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));
  assert.equal(exported.motors[0].kp,100);assert.equal(exported.motors[0].pidDown.kp,777);
  h.event('window','keydown',{code:'KeyQ',key:'q',target:h.node('pidKp')});h.advance();assert.equal(h.value('ui.latest.mode'),0);
  assert.deepEqual(h.physicalWrites,[]);
});

test('old firmware remains readable but cannot accept directional or bulk PID edits',()=>{
  const h=appHarness();h.value('link.connected=true');
  const f=makeFrame({version:'V6',timing:{speedPeriodMs:5,filterPeriodMs:5,cutoffHz:20},frictionMove:[.05,.05,.05]});
  h.value('receive('+JSON.stringify(f)+')');
  assert.equal(h.node('pidApplyBtn').disabled,true);assert.equal(h.node('tuningApplyAllBtn').disabled,true);
  h.node('pidApplyBtn').onclick();assert.deepEqual(h.physicalWrites,[]);
  const profile=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));
  assert.deepEqual(profile.motors.map(m=>[m.kp,m.pidDown.kp]),[[0,0],[0,0],[0,0]]);
});


test('actual app: response floating and all window resizes preserve speed control, six-bank draft and paused history',()=>{
  const h=appHarness(true);h.node('demoBtn').onclick();
  h.node('speedTarget1').value='-3';h.node('speedStartBtn').onclick();h.advance(3);
  h.node('pidKp').value='17.123';h.node('pidKp').oninput();
  h.node('responsePauseBtn').onclick();
  const run=h.value('speedTest.run'),snapshot=h.value('ui.charts.response.snapshot');
  h.value('sendCommand = () => {throw new Error("Window operation must not send commands");}');
  h.node('responseOpenBtn').onclick();
  h.event('responseHandle','mousedown',{button:0,clientX:130,clientY:100,target:h.node('responseHandle')});
  h.event('document','mousemove',{clientX:230,clientY:180});
  h.event('document','mouseup',{});
  for(const id of ['fixedWindow','speedParamsWindow','speedTrackWindow','responsePanel'])
    h.event(id+'Resize','keydown',{key:'ArrowRight',shiftKey:true});
  h.node('responseCloseBtn').onclick();
  assert.equal(h.value('speedTest.run'),run);assert.equal(h.value('ui.latest.mode'),5);
  assert.equal(h.value('ui.demo.targets[0]'),-3);assert.equal(h.value('ui.tuningDraft.motors[0].kp'),17.123);
  assert.equal(h.value('ui.charts.response.paused'),true);assert.equal(h.value('ui.charts.response.snapshot'),snapshot);
  assert.equal(h.node('responsePanel').hidden,false);assert.deepEqual(h.physicalWrites,[]);
});


test('angle UI: six outer PID drafts, atomic config readback, cascade convergence, speed limit, charts pause and close semantics, Q',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();h.value('sendCommand("P,0")');h.advance();
  for(let motor=1;motor<=3;motor++)for(const direction of [-1,1]){
    h.node('pidMotorSelect').value=String(motor);h.node('pidMotorSelect').onchange();
    h.node('pidDirectionSelect').value=String(direction);h.node('pidDirectionSelect').onchange();
    h.node('pidKp').value='100';h.node('pidKi').value='100';h.node('pidKd').value='0';h.node('pidFriction').value='0';h.node('pidApplyBtn').onclick();
    h.node('positionMotorSelect').value=String(motor);h.node('positionMotorSelect').onchange();
    h.node('positionDirectionSelect').value=String(direction);h.node('positionDirectionSelect').onchange();
    h.node('positionKp').value=String(direction<0?4:6);h.node('positionKi').value='0';h.node('positionKd').value='0';h.node('positionMaxSpeed').value='10';
    h.node('positionApplyBtn').onclick();h.advance();assert.equal(h.value('ui.pendingPosition'),null);
  }
  assert.equal(h.value('ui.latest.mode'),1);assert.equal(h.value('ui.demo.targets.every(v=>v===0)'),true);
  const exported=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));
  assert.equal(exported.version,3);assert.deepEqual(exported.position.pid.map(p=>[p.up.kp,p.down.kp,p.maxSpeed]),[[4,6,10],[4,6,10],[4,6,10]]);
  h.node('positionTarget1').value='5';h.node('positionTarget2').value='20';h.node('positionTarget3').value='12.77';
  h.node('positionStartBtn').onclick();h.advance(100);
  assert.equal(h.value('ui.latest.mode'),6);assert.equal(h.value('angleTest.run.phase'),'tracking');
  assert.equal(h.node('speedStartBtn').disabled,true);assert.equal(h.node('fixedApplyBtn').disabled,true);
  assert.ok(h.value('ui.history.filter(f=>f.mode===6).every(f=>f.speedTarget.every(v=>Math.abs(v)<=10))'));
  assert.ok(h.value('Math.abs(ui.latest.angles[0]-5)')<=.5);assert.ok(h.value('Math.abs(ui.latest.angles[1]-20)')<=.5);
  assert.equal(h.value('ui.latest.angleSettling.state.every(s=>s===2)'),true);
  h.node('positionFixedY').checked=true;h.node('positionYMin').value='-.1';h.node('positionYMax').value='.1';h.node('positionPlotMode').value='error';
  h.node('positionTimeWindow').value='2';h.node('positionPauseBtn').onclick();
  const last=h.value('ui.charts.position.snapshot.at(-1).t');h.advance(3);
  assert.equal(h.value('ui.charts.position.snapshot.at(-1).t'),last);assert.ok(h.value('ui.history.at(-1).t')>last);
  assert.equal(h.value('angleTest.run.phase'),'tracking');assert.match(h.node('positionStats').textContent,/RMS/);
  h.node('positionExpandBtn').onclick();assert.equal(h.value('ui.charts.position.expanded'),true);
  h.node('positionTrackCloseBtn').onclick();assert.ok(h.value('angleTest.run'));h.node('positionTrackOpenBtn').onclick();assert.equal(h.node('positionTrackWindow').hidden,false);
  h.node('positionParamsCloseBtn').onclick();h.advance();assert.equal(h.value('angleTest.run'),null);assert.equal(h.value('ui.latest.mode'),0);
  h.value('sendCommand("G")');h.advance();h.node('positionStartBtn').onclick();h.advance();
  h.event('window','keydown',{code:'KeyQ',key:'q',target:h.node('positionKp'),repeat:false});h.advance();
  assert.equal(h.value('angleTest.run'),null);assert.equal(h.value('ui.latest.mode'),0);assert.equal(h.value('ui.latest.total.every(v=>v===0)'),true);
  assert.deepEqual(h.physicalWrites,[]);
});

test('angle form roundtrip keeps all six unsaved banks and targets without starting, only config applied',async()=>{
  const h=appHarness(true);h.node('demoBtn').onclick();
  for(let motor=1;motor<=3;motor++)for(const direction of [-1,1]){
    h.node('positionMotorSelect').value=String(motor);h.node('positionMotorSelect').onchange();
    h.node('positionDirectionSelect').value=String(direction);h.node('positionDirectionSelect').onchange();
    h.node('positionKp').value=String(motor+(direction>0?.5:0));h.node('positionKi').value='.1';h.node('positionKd').value='.02';h.node('positionMaxSpeed').value=String(motor*5);
  }
  h.node('positionTarget1').value='-20';h.node('positionTarget2').value='-30';h.node('positionTarget3').value='-40';
  const c=JSON.parse(JSON.stringify(h.value('buildTuningExport("draft")')));
  assert.deepEqual(c.position.pid.map(p=>p.up.kp),[1,2,3]);assert.deepEqual(c.position.pid.map(p=>p.down.kp),[1.5,2.5,3.5]);
  h.value('loadTuningFile('+JSON.stringify(c)+')');h.advance();assert.equal(h.value('ui.latest.mode'),1);assert.equal(h.value('ui.latest.anglePid[2].up.kp'),0);
  await h.node('tuningApplyAllBtn').onclick();h.advance();assert.equal(h.value('ui.pendingTuning'),null);
  assert.equal(h.value('ui.latest.mode'),1);assert.equal(h.value('ui.demo.targets.every(v=>v===0)'),true);assert.equal(h.value('ui.demo.angleTargets[0]'),12.77);
  assert.equal(h.value('ui.latest.anglePid[2].down.kp'),3.5);assert.equal(Number(h.node('positionTarget3').value),-40);
  h.node('positionParamsOpenBtn').onclick();const title=h.node('positionParamsHandle');
  h.event('positionParamsHandle','mousedown',{button:0,clientX:130,clientY:100,target:title});h.event('document','mousemove',{clientX:330,clientY:220,target:title});
  assert.equal(h.node('positionParamsWindow').style.left,'300px');h.event('document','mouseup',{});
  const width=h.node('positionParamsWindow').getBoundingClientRect().width;
  h.event('positionParamsWindowResize','pointerdown',{pointerId:11,button:0,clientX:500,clientY:400});
  h.event('document','pointermove',{pointerId:11,clientX:550,clientY:450});h.event('document','pointerup',{pointerId:11});
  assert.equal(parseFloat(h.node('positionParamsWindow').style.width),width+50);
  assert.deepEqual(h.physicalWrites,[]);
});

test('angle stop on lost focus and disconnect clears its heartbeat instead of restoring output',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();h.node('positionStartBtn').onclick();h.advance();assert.equal(h.value('ui.latest.mode'),6);
  h.event('window','blur',{});h.advance();assert.equal(h.value('angleTest.run'),null);assert.equal(h.value('ui.latest.mode'),0);
  h.value('sendCommand("G")');h.advance();h.node('positionStartBtn').onclick();h.advance();
  await h.node('disconnectBtn').onclick();assert.equal(h.value('angleTest.run'),null);assert.equal(h.value('ui.demo'),null);
  assert.match(h.node('notice').textContent,/保持急停零输出/);assert.deepEqual(h.physicalWrites,[]);
});


test('angle settling UI: shared settings roundtrip without motion, short pass rejected, settled heartbeat retained and disturbance resumes',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();
  assert.equal(h.value('ui.latest.angleSettling.holdMs'),20);assert.equal(Number(h.node('positionTolerance').value),.5);
  h.node('positionTolerance').value='.25';h.node('positionHoldMs').value='7';h.node('positionSettlingApplyBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.mode'),1);assert.equal(h.value('ui.pendingSettling'),null);
  assert.equal(h.value('ui.latest.angleSettling.toleranceDeg'),.25);assert.equal(h.value('ui.latest.angleSettling.holdMs'),7);
  const c=JSON.parse(JSON.stringify(h.value('buildTuningExport("draft")')));assert.deepEqual(c.position.settling,{toleranceDeg:.25,holdMs:7});
  h.node('positionTolerance').value='0';h.node('positionHoldMs').value='1.5';h.node('positionSettlingApplyBtn').onclick();h.advance();assert.equal(h.value('ui.latest.angleSettling.holdMs'),7);
  h.value('loadTuningFile('+JSON.stringify(c)+')');h.advance();assert.equal(Number(h.node('positionTolerance').value),.25);assert.equal(Number(h.node('positionHoldMs').value),7);
  await h.node('tuningApplyAllBtn').onclick();h.advance();assert.equal(h.value('ui.pendingTuning'),null);assert.equal(h.value('ui.latest.mode'),1);
  h.value('sendCommand("P,0");sendCommand("H,500,20")');h.advance();
  h.node('positionStartBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.mode'),6);assert.equal(h.value('ui.latest.angleSettling.state.every(s=>s===2)'),true);
  assert.ok(h.value('angleTest.run'));assert.equal(h.value('ui.latest.speedTarget.every(v=>v===0)'),true);assert.match(h.node('positionControlState').textContent,/三路成功到位/);
  const target=h.value('ui.demo.angleTargets[0]');
  h.value('ui.demo.angles[0]=ui.demo.angleTargets[0]-.6');h.advance();
  assert.equal(h.value('ui.latest.angleSettling.state[0]'),0);assert.match(h.node('positionSettlingState').textContent,/M1 纠偏中/);
  // Place M1 just inside with outward velocity: a transient crossing cannot count as a settled arrival.
  h.value('ui.demo.angles[0]=ui.demo.angleTargets[0]-.49;ui.demo.speeds[0]=-100');h.advance();
  assert.equal(h.value('ui.latest.angleSettling.state[0]'),0);
  h.value('ui.demo.angles[0]=ui.demo.angleTargets[0];ui.demo.speeds[0]=0;ui.demo.filteredSpeeds[0]=0;ui.demo.rawSpeeds[0]=0');h.advance();
  assert.equal(h.value('ui.latest.angleSettling.state[0]'),2);assert.equal(h.value('ui.demo.angleTargets[0]'),target);assert.ok(h.value('angleTest.run'));
  h.event('window','keydown',{code:'KeyQ',key:'q',repeat:false,target:h.node('positionTolerance')});h.advance();assert.equal(h.value('ui.latest.mode'),0);
  assert.deepEqual(h.physicalWrites,[]);
});


test('old V8 keeps angle control but arrival settings are gated and an imported H config cannot be partially applied',async()=>{
  const h=appHarness();h.value('link.connected=true');
  const f=Core.parseTelemetry(fs.readFileSync(path.join(__dirname,'fixtures/position-v8.txt'),'utf8'));f.mode=1;
  h.value('receive('+JSON.stringify(f)+')');assert.equal(h.node('positionStartBtn').disabled,false);assert.equal(h.node('positionSettlingApplyBtn').disabled,true);
  h.node('positionSettlingApplyBtn').onclick();assert.deepEqual(h.physicalWrites,[]);
  const c=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));c.position.settling={toleranceDeg:.5,holdMs:20};
  h.value('loadTuningFile('+JSON.stringify(c)+')');await h.node('tuningApplyAllBtn').onclick();assert.deepEqual(h.physicalWrites,[]);assert.match(h.node('notice').textContent,/V9/);
});

test('minimum outer-loop speed: per-motor shared draft, device ack, JSON roundtrip, floor and settled zero',async()=>{
  const h=appHarness();h.node('demoBtn').onclick();h.advance();
  h.node('positionMinSpeed').value='3';h.node('positionMinSpeed').oninput();h.node('positionKp').value='1';h.node('positionMaxSpeed').value='10';h.node('positionApplyBtn').onclick();h.advance();
  assert.equal(h.value('ui.latest.anglePid[0].minSpeed'),3);assert.equal(h.value('ui.latest.anglePid[1].minSpeed'),0);assert.equal(h.value('ui.pendingPosition'),null);
  h.node('positionDirectionSelect').value='1';h.node('positionDirectionSelect').onchange();assert.equal(Number(h.node('positionMinSpeed').value),3);
  const exported=JSON.parse(JSON.stringify(h.value('buildTuningExport("device")')));assert.equal(exported.position.pid[0].minSpeed,3);
  h.value('loadTuningFile('+JSON.stringify(exported)+')');assert.equal(Number(h.node('positionMinSpeed').value),3);assert.equal(h.value('ui.latest.mode'),1);
  const step=(target,kp=1)=>h.value('positionPidStep({integral:0,previousError:0,initialized:false},{kp:'+kp+',ki:0,kd:0},'+target+',0,.005,10,3)');
  assert.equal(step(1),3);assert.equal(step(-1),-3);assert.equal(step(100),10);assert.equal(step(1,0),0);
  // Remove the capability from a simulated old device: reject the complete file before the first command.
  h.value('ui.demo=null;link.connected=true;link.send=async line=>{window.sentConfigs.push(line);return true;}');h.value('window.sentConfigs=[];ui.latest.anglePid.forEach(p=>delete p.minSpeed)');
  await h.node('tuningApplyAllBtn').onclick();assert.equal(h.value('window.sentConfigs.length'),0);assert.match(h.node('notice').textContent,/V11/);
});
