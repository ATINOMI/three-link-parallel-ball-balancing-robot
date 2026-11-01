const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const C=require('../core.js'),T=require('../tuning.js'),{SerialLink}=require('../serial.js');
const wire=()=>fs.readFileSync(path.join(__dirname,'fixtures/position-v8.txt'),'utf8');
test('actual V8 frame decodes all 100 fields with physical units, legacy fields and six outer banks',()=>{
  const text=wire(),f=C.parseTelemetry(text);assert.equal(text.trim().split(',').length,100);assert.equal(f.version,'V8');assert.equal(f.mode,6);
  assert.deepEqual(f.angleTarget,[2.77,20.77,12.77]);assert.deepEqual(f.anglePid.map(p=>p.up.kp),[1,3,5]);assert.deepEqual(f.anglePid.map(p=>p.down.kp),[2,4,6]);
  assert.deepEqual(f.anglePid.map(p=>p.maxSpeed),[50,51,52]);assert.ok(f.speedTarget[0]<0 && f.speedTarget[1]>0);
  const frames=[],r=new C.LineReader(line=>frames.push(C.parseTelemetry(line)));for(let i=0;i<text.length;i+=5)r.push(text.slice(i,i+5));assert.deepEqual(frames,[f]);
  assert.equal(C.parseTelemetry(text.replace('V8','V7')),null);
});
test('angle heartbeat, zero target, exact device confirmation, stale/arming timeout, emergency and unexpected mode',()=>{
  let now=0;const sent=[],reasons=[],loop=new C.AngleLoop({send:s=>sent.push(s),onFinish:s=>reasons.push(s),now:()=>now});
  const f=C.parseTelemetry(wire());loop.accept({...f,mode:1,anglePid:null});assert.throws(()=>loop.start([0,0,0]),/V8/);
  loop.accept({...f,mode:1});loop.start([-20,0,12.77]);assert.equal(sent.at(-1),'A,-20000,0,12770');assert.equal(loop.run.kind,'position');
  loop.accept({...f,angleTarget:[-19,0,12.77]});assert.equal(loop.run.phase,'arming');loop.accept({...f,angleTarget:[-20,0,12.77]});assert.equal(loop.run.phase,'tracking');
  now=50;loop.tick();assert.equal(sent.at(-1),'A,-20000,0,12770');loop.start([0,0,0]);assert.equal(sent.at(-1),'A,0,0,0');
  const count=sent.length;loop.accept({...f,mode:0});assert.equal(loop.run,null);assert.equal(sent.length,count);
  loop.accept({...f,mode:1});loop.start([1,2,3]);now+=501;loop.tick();assert.equal(sent.at(-1),'S');assert.match(reasons.at(-1),/角度/);
  loop.accept({...f,mode:1});loop.start([1,2,3]);now+=1501;loop.accept({...f,mode:1});loop.tick();assert.equal(sent.at(-1),'S');
  loop.accept({...f,mode:1});assert.throws(()=>loop.start([181,0,0]));loop.start([1,2,3]);loop.accept({...f,angleTarget:[1,2,3]});loop.accept({...f,mode:5});assert.equal(loop.run,null);
});
test('queued angle targets are invalidated by emergency; config never clears stop or resumes on disconnect',async()=>{
  const writes=[],link=new SerialLink({onFrame(){},onStatus(){},onLegacy(){},onError(e){throw e;}});link.connected=true;
  link.writer={async write(b){writes.push(new TextDecoder().decode(b).trim());},releaseLock(){}};link.port={async close(){}};
  link.send('A,-10000,0,0');link.send('A,-20000,0,0');await link.send('S');await link.send('J,1,-1,1000,0,0,50000');
  assert.equal(link.emergencyLatched,true);await link.disconnect();assert.deepEqual(writes,['S','J,1,-1,1000,0,0,50000']);
});
test('v3 profiles preserve angle and speed banks; config commands never send motion targets; v1/v2 files remain compatible',()=>{
  const f=C.parseTelemetry(wire()),c=T.fromFrame(f);assert.equal(c.version,3);assert.ok(T.matchesFrame(c,f));
  const roundtrip=T.normalize(JSON.parse(JSON.stringify(c)));assert.deepEqual(roundtrip,c);const lines=T.commands(c);
  assert.equal(lines.filter(s=>s.startsWith('J,')).length,6);assert.ok(lines.includes('J,2,1,4000,11,2,51000'));
  assert.equal(lines.some(s=>/^(A|V|D|F|B|G|S)(,|$)/.test(s)),false);
  c.position.pid[1].down.kp+=1;assert.equal(T.matchesFrame(c,f),false);c.position.pid[1].maxSpeed=0;assert.throws(()=>T.normalize(c));
  const old=JSON.parse(fs.readFileSync(path.join(__dirname,'../final parameter.json'),'utf8')),before=JSON.stringify(old),migrated=T.normalize(old);
  assert.equal(migrated.position,null);assert.equal(migrated.motors[1].pidDown.kp,8.4);assert.equal(JSON.stringify(old),before);assert.equal(T.commands(migrated).some(s=>s.startsWith('J,')),false);
});


test('actual V9 frame preserves V8 fields and reports range, continuous dwell and independent arrival states',()=>{
  const text=fs.readFileSync(path.join(__dirname,'fixtures/angle-settling-v9.txt'),'utf8'),f=C.parseTelemetry(text);
  assert.equal(text.trim().split(',').length,105);assert.equal(f.version,'V9');assert.equal(f.mode,6);
  assert.deepEqual(f.angleSettling,{toleranceDeg:.5,holdMs:20,state:[2,1,0]});assert.deepEqual(f.angleTarget,[10,10,10]);
  assert.equal(f.speedTarget[0],0);assert.ok(f.anglePid);const frames=[],r=new C.LineReader(line=>frames.push(C.parseTelemetry(line)));
  for(let i=0;i<text.length;i+=11)r.push(text.slice(i,i+11));assert.deepEqual(frames,[f]);
  assert.equal(C.parseTelemetry(text.replace('V9','V8')),null);
  const c=T.fromFrame(f);assert.deepEqual(c.position.settling,{toleranceDeg:.5,holdMs:20});assert.ok(T.matchesFrame(c,f));
  assert.ok(T.commands(c).includes('H,500,20'));assert.equal(T.commands(c).some(s=>/^(A|V|D|F|B|G|S)(,|$)/.test(s)),false);
  c.position.settling.holdMs=21;assert.equal(T.matchesFrame(c,f),false);
  const old=T.fromFrame(C.parseTelemetry(wire()));assert.equal(old.position.settling,null);assert.equal(T.commands(old).some(s=>s.startsWith('H,')),false);
});
test('arrival settings validate ranges and integer milliseconds and survive full JSON roundtrip',()=>{
  const c=T.fromFrame(C.parseTelemetry(wire()));c.position.settling={toleranceDeg:.25,holdMs:7};
  const round=T.normalize(JSON.parse(JSON.stringify(c)));assert.deepEqual(round,c);assert.ok(T.commands(round).includes('H,250,7'));
  for(const bad of [{toleranceDeg:0,holdMs:7},{toleranceDeg:11,holdMs:7},{toleranceDeg:.5,holdMs:0},{toleranceDeg:.5,holdMs:7.5},{toleranceDeg:.5,holdMs:5001}]){
    c.position.settling=bad;assert.throws(()=>T.normalize(c));
  }
});
