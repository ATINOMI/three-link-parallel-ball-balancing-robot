const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const C=require('../core.js'),T=require('../tuning.js');
test('V10 preserves V9 telemetry, full tuning roundtrip and no motion on config apply',()=>{
  const v9=fs.readFileSync(path.join(__dirname,'fixtures/angle-settling-v9.txt'),'utf8');
  const old=C.parseTelemetry(v9),f=C.parseTelemetry(v9.replace('V9,','V10,'));
  assert.deepEqual({...f,version:'V9'},old);assert.equal(f.version,'V10');
  const config=T.fromFrame(f);assert(T.matchesFrame(config,f));
  assert.equal(T.commands(config).some(s=>/^(A|V|D|F|B|G|S)(,|$)/.test(s)),false);
});
