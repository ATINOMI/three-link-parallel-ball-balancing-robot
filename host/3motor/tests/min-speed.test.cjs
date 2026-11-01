const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const C=require('../core.js'),T=require('../tuning.js');
const oldWire=()=>fs.readFileSync(path.join(__dirname,'fixtures/angle-settling-v9.txt'),'utf8').trim();
const wire=()=>oldWire().replace(/^V9/, 'V11')+',1250,2500,3750';
test('V11 appends three min speeds and preserves all V9 values with split-frame decoding',()=>{
  const frame=C.parseTelemetry(wire()),old=C.parseTelemetry(oldWire());assert.equal(frame.version,'V11');
  assert.deepEqual(frame.anglePid.map(p=>p.minSpeed),[1.25,2.5,3.75]);
  const stripped={...frame,version:'V9',anglePid:frame.anglePid.map(({minSpeed,...p})=>p)};assert.deepEqual(stripped,old);
  const frames=[],reader=new C.LineReader(s=>frames.push(C.parseTelemetry(s)));for(let i=0;i<wire().length;i+=7)reader.push(wire().slice(i,i+7));reader.push('\n');assert.deepEqual(frames,[frame]);
  assert.equal(C.parseTelemetry(wire()+',0'),null);
});
test('V11 bounds survive JSON/export/ack; legacy files and frames still send old J',()=>{
  const f=C.parseTelemetry(wire()),c=T.fromFrame(f);assert.deepEqual(T.normalize(JSON.parse(JSON.stringify(c))),c);assert(T.matchesFrame(c,f));
  const commands=T.commands(c).filter(s=>s.startsWith('J,'));assert.equal(commands.length,6);assert(commands.filter(s=>s.startsWith('J,1,')).every(s=>s.endsWith(',1250')));
  assert.equal(T.commands(c).some(s=>/^(A|V|D|F|B|G|S)(,|$)/.test(s)),false);
  c.position.pid[0].minSpeed=2;assert.equal(T.matchesFrame(c,f),false);
  c.position.pid[0].minSpeed=-1;assert.throws(()=>T.normalize(c));c.position.pid[0].minSpeed=181;assert.throws(()=>T.normalize(c));
  const old=T.fromFrame(C.parseTelemetry(oldWire()));assert.equal('minSpeed' in old.position.pid[0],false);assert(T.commands(old).filter(s=>s.startsWith('J,')).every(s=>s.split(',').length===7));
});
