'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const T=require('../tuning.js'),Core=require('../core.js');

test('JSON roundtrip keeps all three independent motors, timing and targets; config commands never start motion',()=>{
  const c=T.defaults();c.kg=1.234;c.motors.forEach((m,i)=>Object.assign(m,{kp:10+i,ki:.1+i,kd:.002,frictionRaw:100+i,outputScale:1+.2*i,frictionMoveDeg:.05+.01*i}));
  c.speedTargets=[-3,2,0];c.timing={speedPeriodMs:2,filterPeriodMs:10,cutoffHz:20};
  const result=T.normalize(JSON.parse(JSON.stringify(c)));
  assert.deepEqual(result,T.normalize(c));assert.equal(result.motors[1].frictionMoveDeg,.06);const commands=T.commands(result);
  assert.deepEqual(commands.slice(0,3),['P,1234','R,1000,1200,1400','U,2,10']);
  assert.ok(commands.includes('K,3,-1,12000,2100,2,102'));assert.ok(commands.includes('T,2,60'));
  assert.equal(commands.some(c=>/^(V|D|F|B|G|S)(,|$)/.test(c)),false);
});
test('file validation rejects wrong format, duplicates, non-finite/range errors before use',()=>{
  assert.throws(()=>T.normalize({}),/不.*支持/);
  let c=T.defaults();c.motors[2].id=2;assert.throws(()=>T.normalize(c),/重复/);
  c=T.defaults();c.motors[0].kp=NaN;assert.throws(()=>T.normalize(c));
  c=T.defaults();c.timing.filterPeriodMs=25;assert.throws(()=>T.normalize(c));
  c=T.defaults();c.motors[1].frictionRaw=1.5;assert.throws(()=>T.normalize(c),/整数/);
});
test('frequency quantization and default filter gain/phase match the bilinear Butterworth design',()=>{
  assert.equal(T.periodFromHz(333),3);assert.equal(T.periodFromHz(100,true),10);
  assert.throws(()=>T.periodFromHz(40,true));
  const c=T.designFilter(20,5);assert.ok(Math.abs(c.b0-.06745527388907191)<1e-12);
  const at5=T.response(20,5,5),at10=T.response(20,5,10),at20=T.response(20,5,20);
  assert.ok(Math.abs(at5.phase+19.996)<.001);assert.ok(Math.abs(at10.phase+42.121)<.001);
  assert.ok(Math.abs(at20.phase+90)<1e-10);assert.ok(Math.abs(at20.gainDb+3.0103)<.0001);
  const state=T.resetFilter(c,123);for(let i=0;i<50;i++)assert.ok(Math.abs(T.filterStep(state,c,123)-123)<1e-10);
});
test('actual firmware V6 frame returns periods and cutoff; device export matches every applied parameter',()=>{
  const wire=fs.readFileSync(path.join(__dirname,'fixtures/butterworth-v6.txt'),'utf8');
  const f=Core.parseTelemetry(wire);assert.equal(wire.trim().split(',').length,58);assert.equal(f.version,'V6');
  assert.deepEqual(f.timing,{speedPeriodMs:5,filterPeriodMs:5,cutoffHz:20});
  const c=T.fromFrame(f);assert.ok(T.matchesFrame(c,f));c.motors[0].kp+=1;assert.equal(T.matchesFrame(c,f),false);
});


test('v1 imports duplicate shared PID while v2 keeps independently validated up/down settings',()=>{
  const old=T.defaults();old.version=1;old.motors[0].kp=123;delete old.motors[0].pidDown;
  const migrated=T.normalize(old);assert.equal(migrated.version,3);assert.equal(migrated.motors[0].pidDown.kp,123);
  migrated.motors[0].pidDown={kp:456,ki:7,kd:.008};
  const c=T.normalize(JSON.parse(JSON.stringify(migrated)));
  assert.equal(c.motors[0].kp,123);assert.deepEqual(c.motors[0].pidDown,{kp:456,ki:7,kd:.008});
  assert.ok(T.commands(c).includes('K,1,1,456000,7000,8,0'));
  c.motors[0].pidDown.ki=NaN;assert.throws(()=>T.normalize(c));
});

test('real firmware V7 telemetry reads all six PID banks and split frames, file export/ack compares inactive bank',()=>{
  const wire=fs.readFileSync(path.join(__dirname,'fixtures/directional-pid-v7.txt'),'utf8');
  assert.equal(wire.trim().split(',').length,76);const f=Core.parseTelemetry(wire);assert.equal(f.version,'V7');
  assert.deepEqual(f.pidDirectional.map(p=>p.up.kp),[100,300,500]);assert.deepEqual(f.pidDirectional.map(p=>p.down.kp),[200,400,600]);
  const frames=[],r=new Core.LineReader(line=>frames.push(Core.parseTelemetry(line)));
  for(let i=0;i<wire.length;i+=7)r.push(wire.slice(i,i+7));assert.equal(frames.length,1);assert.deepEqual(frames[0],f);
  const c=T.fromFrame(f);assert.ok(T.matchesFrame(c,f));c.motors[1].pidDown.ki+=.1;assert.equal(T.matchesFrame(c,f),false);
  assert.equal(Core.parseTelemetry(wire.replace('V7','V6')),null);
});
