'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const Core = require('../core.js');
const { SerialLink } = require('../serial.js');
const Model = require('../drawing.js');

function rig() {
  let now = 1000, serial = 1;
  const commands = [], results = [];
  const engine = new Core.BreakawayTest({ now: () => now, send: cmd => commands.push(cmd), onFinish: r => results.push(r) });
  const frame = (overrides = {}) => ({ ms: serial++, mode: 1, motor: 0, kg: 1.4, fresh: 7,
    angles: [0, 12, -5], pose: [0, 0, .09], base: [-200, -210, -220], delta: [0, 0, 0],
    total: [-200, -210, -220], feedback: [-199, -209, -219], ...overrides });
  const feed = overrides => { const f = frame(overrides); engine.accept(f); return f; };
  feed();
  return { engine, commands, results, feed, frame, advance: ms => { now += ms; } };
}
const options = { limit: .5, ramp: .1, threshold: .5, duration: 15 };

test('V2 has 24 fields; unit conversion; split frames and overlong recovery', () => {
  const wire = 'V2,100,2,1400,2,7,0,-1000,2000,1,-2,89459,-200,-201,-202,0,82,0,-200,-119,-202,-199,-120,-201';
  const f = Core.parseTelemetry(wire);
  assert.deepEqual(f.angles, [0, -1, 2]); assert.equal(f.pose[2], .089459);
  assert.deepEqual(f.base, [-200, -201, -202]); assert.deepEqual(f.delta, [0, 82, 0]);
  assert.deepEqual(f.total, [-200, -119, -202]); assert.equal(f.motor, 2);
  assert.equal(Core.parseTelemetry(wire.replace('V2', 'T')), null);
  const seen = [], lines = new Core.LineReader(line => { const f = Core.parseTelemetry(line); if (f) seen.push(f); });
  lines.push(wire.slice(0, 31)); assert.equal(seen.length, 0);
  lines.push(wire.slice(31) + '\r\n' + 'x'.repeat(600) + '\n' + wire + '\n');
  assert.equal(seen.length, 2); assert.equal(Core.toRaw(.5), 410);
});

test('zero-delta acknowledgement precedes ramp; ramp is based on actual elapsed time', () => {
  const r = rig(); r.engine.start(1, -1, options);
  assert.deepEqual(r.commands, ['D,1,-1,0']);
  r.advance(100); r.engine.tick(); assert.equal(r.commands.at(-1), 'D,1,-1,0');
  r.feed({ mode: 2, motor: 1 });
  r.advance(300); r.engine.tick(); assert.equal(r.commands.at(-1), `D,1,-1,${Core.toRaw(-.03)}`);
  assert.equal(r.engine.run.startAngle, 0);
  r.engine.finish(); assert.equal(r.commands.at(-1), 'G');
});

test('motion caused by zero-increment feedforward is measured from the pre-command angle', () => {
  const r = rig(); r.engine.start(1, -1, options);
  r.advance(20);
  r.feed({ mode: 2, motor: 1, angles: [-.6, 12, -5], delta: [0, 0, 0] });
  assert.equal(r.engine.run.startAngle, 0);
  r.feed({ mode: 2, motor: 1, angles: [-.7, 12, -5], delta: [0, 0, 0] });
  r.feed({ mode: 2, motor: 1, angles: [-.8, 12, -5], delta: [0, 0, 0] });
  assert.equal(r.results[0].reason, '检测到起动');
  assert.equal(r.results[0].delta, 0);
  assert.equal(r.results[0].startAngle, 0);
  assert.equal(r.commands.at(-1), 'G');
});

test('directional displacement needs three distinct consecutive frames; record actual first crossing', () => {
  const r = rig(); r.engine.start(2, 1, options); r.feed({ mode: 2, motor: 2 });
  r.feed({ mode: 2, motor: 2, angles: [0, 12.6, -5] });
  r.feed({ mode: 2, motor: 2, angles: [0, 12.49, -5] }); // transient is not a result
  assert.equal(r.results.length, 0);
  const sample = { mode: 2, motor: 2, angles: [0, 12.55, -5],
    base: [-200, -250, -220], delta: [0, 80, 0], total: [-200, -170, -220] };
  const first = r.feed(sample);
  r.engine.accept(first); r.engine.accept(first); assert.equal(r.results.length, 0);
  r.feed({ ...sample, delta: [0, 90, 0], total: [-200, -160, -220] });
  r.feed({ ...sample, delta: [0, 100, 0], total: [-200, -150, -220] });
  assert.equal(r.results.length, 1); assert.equal(r.results[0].reason, '检测到起动');
  assert.equal(r.results[0].delta, Core.toAmp(80));
  assert.equal(r.results[0].baseline, Core.toAmp(-250));
  assert.equal(r.results[0].total, Core.toAmp(-170));
  assert.equal(r.commands.at(-1), 'G'); assert.equal(r.engine.run, null);
});

test('opposite movement is not mistaken for an onset in the requested direction', () => {
  const r = rig(); r.engine.start(1, -1, options); r.feed({ mode: 2, motor: 1 });
  for (let i = 0; i < 3; i++) r.feed({ mode: 2, motor: 1, angles: [.6, 12, -5] });
  assert.equal(r.results[0].reason, '检测到反向位移'); assert.equal(r.commands.at(-1), 'G');
});

test('stale telemetry withdraws increment; device emergency stop never emits G', () => {
  const r = rig(); r.engine.start(1, 1, options); r.feed({ mode: 2, motor: 1 });
  r.advance(501); r.engine.tick(); assert.equal(r.commands.at(-1), 'G');
  assert.equal(r.results[0].reason, '遥测超时');
  const s = rig(); s.engine.start(1, 1, options); s.feed({ mode: 0 });
  assert.deepEqual(s.commands, ['D,1,1,0']); assert.equal(s.results[0].reason, '设备已急停');
});

test('duration cap and missing zero-delta confirmation return to gravity', () => {
  const r = rig(); r.engine.start(1, 1, { ...options, duration: 1 }); r.feed({ mode: 2, motor: 1 });
  r.advance(1001); r.feed({ mode: 2, motor: 1 }); r.engine.tick();
  assert.equal(r.results[0].reason, '达到最长时间'); assert.equal(r.commands.at(-1), 'G');
  const s = rig(); s.engine.start(1, -1, options);
  s.advance(1501); s.feed(); s.engine.tick();
  assert.equal(s.results[0].reason, '设备未确认测试'); assert.equal(s.commands.at(-1), 'G');
});

test('serial G/S invalidate queued increments; disconnect after S must not resume gravity', async () => {
  const writes = [];
  const link = new SerialLink({ onFrame() {}, onStatus() {}, onError(e) { throw e; }, onLegacy() {} });
  link.connected = true;
  link.writer = { write: async bytes => writes.push(new TextDecoder().decode(bytes).trim()), releaseLock() {} };
  link.port = { close: async () => {} };
  link.send('D,1,1,100'); link.send('D,1,1,200'); await link.send('G');
  assert.deepEqual(writes, ['G']);
  link.send('D,2,-1,-200'); link.send('F,2,0,-400'); await link.send('S');
  assert.deepEqual(writes, ['G', 'S']); assert.equal(link.emergencyLatched, true);
  await link.disconnect(); assert.deepEqual(writes, ['G', 'S']);
});

test('an already in-flight increment is followed by G, never by obsolete queued increments', async () => {
  const writes = []; let release;
  const gate = new Promise(resolve => { release = resolve; });
  const link = new SerialLink({ onFrame() {}, onStatus() {}, onError(e) { throw e; }, onLegacy() {} });
  link.connected = true;
  link.writer = { async write(bytes) { const line = new TextDecoder().decode(bytes).trim(); writes.push(line); if (line.startsWith('D')) await gate; } };
  link.send('D,1,1,10'); await Promise.resolve();
  link.send('D,1,1,20'); const stopped = link.send('G'); release(); await stopped;
  assert.deepEqual(writes, ['D,1,1,10', 'G']);
});

test('actual app offline flow reaches threshold, withdraws delta, uses direction-gated gravity; no serial writes', () => {
  let time = 0; const intervals = [], nodes = new Map(), physicalWrites = [], windowEvents = [];
  const initial = { motorSelect: '1', limitInput: '.5', rampInput: '.1', thresholdInput: '.5', durationInput: '15', kgInput: '1.4' };
  const makeNode = id => ({ id, value: initial[id] || '', textContent: '', innerHTML: '', disabled: false,
    dataset: {}, style: {}, addEventListener() {}, focus() {}, classList: { toggle() {}, add() {}, remove() {} } });
  const node = id => { if (!nodes.has(id)) nodes.set(id, makeNode(id)); return nodes.get(id); };
  const cards = [0, 1, 2].map(i => ({ ...makeNode('card'), dataset: { motor: String(i) } }));
  const views = ['iso', 'top', 'side'].map(view => ({ ...makeNode('view'), dataset: { view } }));
  const sandbox = {
    console, performance: { now: () => time }, setInterval(fn) { intervals.push(fn); },
    document: { getElementById: node, querySelectorAll: selector => selector === '.motor-card' ? cards : views, addEventListener() {}, hidden: false },
    window: { addEventListener(type, handler, capture) { windowEvents.push({ type, handler, capture }); } },
    MotorDrawing: { ...Model, mechanism() {}, trend() {} },
    MotorSerial: { SerialLink: class {
      constructor() { this.connected = false; this.failed = false; this.emergencyLatched = false; }
      send(line) { physicalWrites.push(line); return Promise.resolve(true); }
    } }
  };
  vm.createContext(sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../core.js'), 'utf8'), sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../tuning.js'), 'utf8'), sandbox);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8'), sandbox);
  node('demoBtn').onclick(); node('upBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.delta[0]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.base[0]', sandbox), 0); // rest does not pre-enable gravity
  assert.equal(vm.runInContext('ui.latest.angles[0]', sandbox), Core.INITIAL_ANGLE_DEG);
  for (let i = 0; i < 100; i++) { time += 50; intervals.forEach(fn => fn()); }
  assert.equal(vm.runInContext('ui.results[0].reason', sandbox), '检测到起动');
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 1);
  assert.equal(vm.runInContext('ui.latest.delta.every(x => x === 0)', sandbox), true);
  assert.equal(vm.runInContext('ui.latest.base.every(x => x === 0)', sandbox), true);
  assert.equal(vm.runInContext('ui.history.some(f => f.delta[0] < 0 && f.base[0] < 0)', sandbox), true);
  assert.deepEqual(physicalWrites, []);
  node('kgInput').value = '1.7'; node('applyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.kg', sandbox), 1.7);
  assert.match(node('paramState').textContent, /1.700/);
  node('motorSelect').value = '3'; node('motorSelect').onchange(); node('downBtn').onclick();
  for (let i = 0; i < 100; i++) { time += 50; intervals.forEach(fn => fn()); }
  assert.equal(vm.runInContext('ui.results[0].reason', sandbox), '检测到起动');
  assert.equal(vm.runInContext('ui.results[0].direction', sandbox), 1);
  assert.equal(vm.runInContext('ui.results[0].baseline', sandbox), 0);
  assert.ok(vm.runInContext('ui.results[0].delta', sandbox) > 0);
  assert.equal(vm.runInContext('ui.latest.delta.every(x => x === 0)', sandbox), true);
  assert.deepEqual(physicalWrites, []);
  node('fixedRawInput').value = '-400'; node('fixedDurationInput').value = '2';
  node('fixedApplyBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 3);
  assert.equal(vm.runInContext('ui.latest.base[2]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), -400);
  assert.equal(node('currentTotal').textContent, '-400');
  for (let i = 0; i < 15; i++) { time += 50; intervals.forEach(fn => fn()); }
  assert.equal(vm.runInContext('fixedTest.run.phase', sandbox), 'holding'); // no displacement stop
  node('fixedRawInput').value = '123'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 123);
  node('fixedRawInput').value = '0'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 3);
  node('fixedGravityInput').checked = true; node('fixedGravityInput').onchange();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 4);
  assert.equal(vm.runInContext('ui.latest.base[2]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 0);
  node('fixedRawInput').value = '-100'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.ok(vm.runInContext('ui.latest.base[2]', sandbox) < 0);
  assert.equal(vm.runInContext('ui.latest.total[2] === ui.latest.base[2] - 100', sandbox), true);
  node('fixedRawInput').value = '100'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.base[2]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 100);
  node('fixedRawInput').value = '0'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  node('fixedGravityInput').checked = false; node('fixedGravityInput').onchange();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 3);
  assert.equal(vm.runInContext('ui.latest.base[2]', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 0);
  node('fixedEndBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 1);
  assert.equal(vm.runInContext('fixedTest.run', sandbox), null);
  assert.equal(node('resultUnit').textContent, 'raw');
  assert.deepEqual(physicalWrites, []);
  node('fixedRawInput').value = '300'; node('fixedApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 3);
  const key = windowEvents.find(event => event.type === 'keydown');
  assert.equal(key.capture, true);
  let prevented = false, stopped = false;
  key.handler({ code: 'KeyQ', key: 'q', repeat: false, target: node('fixedRawInput'),
    preventDefault() { prevented = true; }, stopPropagation() { stopped = true; } });
  time += 50; intervals.forEach(fn => fn());
  assert.equal(prevented && stopped, true);
  assert.equal(vm.runInContext('fixedTest.run', sandbox), null);
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 0);
  assert.equal(vm.runInContext('ui.latest.total.every(x => x === 0)', sandbox), true);
  node('gravityBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  node('upBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.ok(vm.runInContext('test.run', sandbox));
  key.handler({ code: '', key: 'Q', repeat: false, target: node('fixedRawInput'),
    preventDefault() {}, stopPropagation() {} });
  time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('test.run', sandbox), null);
  assert.equal(vm.runInContext('ui.latest.mode', sandbox), 0);
  assert.deepEqual(physicalWrites, []);
  node('gravityBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  node('outputScale1').value = '0.5'; node('outputScale2').value = '1.5'; node('outputScale3').value = '2';
  node('outputScaleApplyBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.pendingScale', sandbox), null);
  assert.match(node('outputScaleState').textContent, /M2 1.500.*M3 2.000/);
  node('motorSelect').value = '3'; node('motorSelect').onchange();
  node('fixedGravityInput').checked = false; node('fixedRawInput').value = '-101';
  node('fixedApplyBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), -202);
  assert.equal(vm.runInContext('fixedTest.run.phase', sandbox), 'holding');
  assert.equal(node('outputScaleApplyBtn').disabled, true);
  node('fixedEndBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  node('outputScale3').value = '5.1'; node('outputScaleApplyBtn').onclick();
  assert.equal(vm.runInContext('ui.demo.outputScale[2]', sandbox), 2);
  node('outputScale3').value = '0'; node('outputScaleApplyBtn').onclick();
  time += 50; intervals.forEach(fn => fn());
  node('fixedApplyBtn').onclick(); time += 50; intervals.forEach(fn => fn());
  assert.equal(vm.runInContext('ui.latest.total[2]', sandbox), 0);
  assert.equal(vm.runInContext('fixedTest.run.phase', sandbox), 'holding');
  const zeroScaleAngle = vm.runInContext('ui.latest.angles[2]', sandbox);
  for (let i=0;i<3;i++) { time += 50; intervals.forEach(fn => fn()); }
  assert.equal(vm.runInContext('ui.latest.angles[2]', sandbox), zeroScaleAngle);
  assert.deepEqual(physicalWrites, []);
});


test('fixed controller repeats the exact raw command, permits update and zero, ignores displacement', () => {
  let now = 1000, ms = 1;
  const commands = [], results = [];
  const fixed = new Core.FixedCurrentTest({ now: () => now, send: c => commands.push(c), onFinish: r => results.push(r) });
  const frame = (value, mode = 3) => ({ ms: ms++, mode, motor: mode === 3 ? 2 : 0, kg: 1.4, fresh: 7,
    angles: [0, 60, 0], pose: [0, 0, .09], base: [-200, 0, -220], delta: [0, value, 0], total: [-200, value, -220], feedback: [-200, value, -220] });
  fixed.accept(frame(0, 1)); fixed.start(2, -400, 2);
  assert.equal(commands.at(-1), 'F,2,0,-400');
  fixed.accept(frame(-400)); assert.equal(fixed.run.phase, 'holding');
  now += 100; fixed.tick(); assert.equal(commands.at(-1), 'F,2,0,-400');
  fixed.accept(frame(-400)); assert.ok(fixed.run);
  fixed.update(123, 1); assert.equal(commands.at(-1), 'F,2,0,123');
  fixed.accept(frame(123)); assert.equal(fixed.run.phase, 'holding');
  fixed.update(0, 1); fixed.accept(frame(0)); assert.equal(commands.at(-1), 'F,2,0,0');
  now += 1000; fixed.accept(frame(0)); fixed.tick();
  assert.equal(commands.at(-1), 'G'); assert.equal(fixed.run, null);
  assert.equal(results[0].kind, 'fixed'); assert.equal(results[0].total, 0);
});

test('fixed command needs exact device confirmation and obeys emergency and stale telemetry', () => {
  let now = 1000;
  const commands = [], results = [];
  const fixed = new Core.FixedCurrentTest({ now: () => now, send: c => commands.push(c), onFinish: r => results.push(r) });
  const normal = { ms: 1, mode: 1, motor: 0, fresh: 7, angles: [0, 0, 0], base: [0, 0, 0], delta: [0, 0, 0], total: [0, 0, 0] };
  fixed.accept(normal);
  assert.throws(() => fixed.start(1, 20000, 1)); assert.equal(fixed.run, null);
  fixed.start(1, 400, 1);
  fixed.accept({ ...normal, mode: 3, motor: 1, total: [399, 0, 0] });
  assert.equal(fixed.run.phase, 'arming');
  now += 1501; fixed.accept(normal); fixed.tick();
  assert.match(results[0].reason, /设备未确认固定指令/); assert.equal(commands.at(-1), 'G');
  fixed.accept(normal); fixed.start(1, 0, 1);
  const count = commands.length; fixed.accept({ ...normal, mode: 0 });
  assert.equal(commands.length, count); assert.equal(fixed.run, null);
  fixed.accept(normal); fixed.start(1, -16384, 1); now += 501; fixed.tick();
  assert.equal(commands.at(-1), 'G'); assert.equal(results.at(-1).reason, '遥测超时');
});


test('fixed feedforward switch is explicit in command and confirmation, and total includes live baseline', () => {
  let now = 1000;
  const commands = [];
  const fixed = new Core.FixedCurrentTest({ now: () => now, send: c => commands.push(c), onFinish() {} });
  const frame = (mode, base, value) => ({ ms: now, mode, motor: mode >= 3 ? 1 : 0, fresh: 7,
    angles: [12.77,12.77,12.77], base: [base,0,0], delta: [value-base,0,0], total: [value,0,0] });
  fixed.accept(frame(1,0,0)); fixed.start(1,100,1,true);
  assert.equal(commands.at(-1),'F,1,1,100');
  fixed.accept(frame(3,0,100)); assert.equal(fixed.run.phase,'arming');
  fixed.accept(frame(4,-250,-150)); assert.equal(fixed.run.phase,'holding'); // actual upward motion may add gravity despite positive raw
  fixed.accept(frame(4,0,100)); assert.equal(fixed.run.phase,'holding');
  fixed.update(0,1); assert.equal(commands.at(-1),'F,1,1,0');
  fixed.accept(frame(4,-260,-260)); assert.equal(fixed.run.phase,'holding'); // zero given does not disable gravity during actual upward motion
  fixed.accept(frame(4,0,0)); assert.equal(fixed.run.phase,'holding');
  fixed.update(-100,1,true); fixed.accept(frame(4,-260,-360)); assert.equal(fixed.run.phase,'holding');
  fixed.update(-16384,1,true); fixed.accept(frame(4,-260,-16384));
  assert.equal(fixed.run.phase,'holding');
  fixed.update(0,1,false); assert.equal(commands.at(-1),'F,1,0,0');
  fixed.accept(frame(3,0,0)); assert.equal(fixed.run.phase,'holding');
});

test('V3 reads three output scales and requested command; saturated zero test and scaled fixed command confirm', () => {
  const line = 'V3,100,4,1400,1,7,12770,12770,12770,0,0,76994,-16384,0,0,0,0,0,-16384,0,0,0,0,0,5000,0,1250,-1000';
  const parsed = Core.parseTelemetry(line);
  assert.equal(parsed.version, 'V3'); assert.deepEqual(parsed.outputScale, [5,0,1.25]);
  assert.equal(parsed.requestedRaw, -1000); assert.equal(parsed.total[0], -16384);
  assert.equal(Core.parseTelemetry(line + ',1'), null);
  let now=0;
  const fixed = new Core.FixedCurrentTest({now:()=>now, send(){}, onFinish(){}});
  const normal = {...parsed,mode:1,motor:0};
  fixed.accept(normal); fixed.start(1,-1000,1,true);
  fixed.accept({...parsed,requestedRaw:-900}); assert.equal(fixed.run.phase,'arming');
  fixed.accept(parsed); assert.equal(fixed.run.phase,'holding');
  fixed.finish(); fixed.accept(normal); fixed.start(2,200,1,false);
  fixed.accept({...parsed,mode:3,motor:2,base:[0,0,0],total:[0,0,0],requestedRaw:200});
  assert.equal(fixed.run.phase,'holding'); // zero output scale still confirms the given command
  const ramp = new Core.BreakawayTest({now:()=>now,send(){},onFinish(){}});
  ramp.accept(normal); ramp.start(1,-1,options);
  ramp.accept({...parsed,mode:2,requestedRaw:-100}); assert.equal(ramp.run.phase,'arming');
  ramp.accept({...parsed,mode:2,requestedRaw:0}); assert.equal(ramp.run.phase,'ramping');
});
