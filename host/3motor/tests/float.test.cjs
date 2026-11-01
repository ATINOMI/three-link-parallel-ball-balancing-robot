'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const app = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
const start = app.indexOf('const fixedWindow =');
const end = app.indexOf("window.addEventListener('blur', () => finishActive", start);
assert.ok(start >= 0 && end > start);
const code = app.slice(start, end);
function createHarness(width = 1000, height = 800, headerBottom = 0) {
  const nodes = new Map(), docEvents = {}, windowEvents = {};
  let finished = 0;
  function node(id) {
    if (!nodes.has(id)) {
      const events = {};
      const value = { id, style: {}, hidden: false, events, value: '1',
        classList: { add() {}, remove() {} }, focus() {},
        closest: () => id.includes('Btn') ? value : null,
        addEventListener(type, handler) { (events[type] ||= []).push(handler); },
        getBoundingClientRect() {
          return { left: this.style.left === undefined ? 100 : parseFloat(this.style.left),
            top: this.style.top === undefined ? 80 : parseFloat(this.style.top),
            width: 400, height: id === 'fixedWindowHandle' ? 70 : 600 };
        }
      };
      nodes.set(id, value);
    }
    return nodes.get(id);
  }
  const sandbox = { $: node, fixedTest: { finish() { finished++; } }, renderReadings() {}, draw() {},
    document: { querySelector: () => ({ getBoundingClientRect: () => ({bottom: headerBottom}) }), addEventListener(type, handler) { (docEvents[type] ||= []).push(handler); } },
    window: { innerWidth: width, innerHeight: height,
      addEventListener(type, handler) { (windowEvents[type] ||= []).push(handler); } }
  };
  vm.createContext(sandbox); vm.runInContext(code, sandbox);
  const handle = node('fixedWindowHandle'), panel = node('fixedWindow');
  function emit(target, type, data = {}) {
    const event = { type, button: 0, clientX: 130, clientY: 100, target: handle, preventDefault() {}, ...data };
    const events = target === 'document' ? docEvents : target === 'window' ? windowEvents : node(target).events;
    (events[type] || []).forEach(handler => handler(event));
  }
  return { node, emit, handle, panel, finished: () => finished };
}

test('ordinary mouse drag continues outside header and stops on document mouseup', () => {
  const h = createHarness();
  h.emit('fixedWindowHandle', 'mousedown');
  h.emit('document', 'mousemove', { clientX: 230, clientY: 170, target: {} });
  assert.equal(h.panel.style.left, '200px'); assert.equal(h.panel.style.top, '150px');
  h.emit('document', 'mouseup');
  h.emit('document', 'mousemove', { clientX: 500, clientY: 400 });
  assert.equal(h.panel.style.left, '200px');
});

test('pointer drag needs no pointer capture; compatibility mouse event does not reset origin', () => {
  const h = createHarness(); // no setPointerCapture API exists in this DOM
  h.emit('fixedWindowHandle', 'pointerdown', { pointerId: 7 });
  h.emit('fixedWindowHandle', 'mousedown', { clientX: 140 });
  h.emit('document', 'pointermove', { pointerId: 8, clientX: 330 });
  assert.equal(h.panel.style.left, undefined);
  h.emit('document', 'pointermove', { pointerId: 7, clientX: 230, clientY: 180 });
  assert.equal(h.panel.style.left, '200px'); assert.equal(h.panel.style.top, '160px');
  h.emit('window', 'blur');
  h.emit('document', 'pointermove', { pointerId: 7, clientX: 400 });
  assert.equal(h.panel.style.left, '200px');
});

test('window taller than viewport is still draggable; title remains reachable', () => {
  const h = createHarness(500, 400);
  h.emit('fixedWindowHandle', 'mousedown');
  h.emit('document', 'mousemove', { clientX: 220, clientY: 200 });
  assert.equal(h.panel.style.left, '190px'); assert.equal(h.panel.style.top, '180px');
  h.emit('document', 'mousemove', { clientX: -5000, clientY: 5000 });
  assert.equal(h.panel.style.left, '-300px'); assert.equal(h.panel.style.top, '322px');
});

test('close button does not begin dragging; closing finishes fixed test and reopening does not send commands', () => {
  const h = createHarness();
  h.emit('fixedWindowHandle', 'pointerdown', { pointerId: 1, target: h.node('fixedCloseBtn') });
  h.emit('document', 'pointermove', { pointerId: 1, clientX: 400 });
  assert.equal(h.panel.style.left, undefined);
  h.node('fixedCloseBtn').onclick();
  assert.equal(h.panel.hidden, true); assert.equal(h.finished(), 1);
  h.node('fixedOpenBtn').onclick();
  assert.equal(h.panel.hidden, false); assert.equal(h.finished(), 1);
});


test('dragged fixed window title stays below the sticky connection and emergency bar', () => {
  const h = createHarness(1000, 800, 84);
  h.emit('fixedWindowHandle', 'mousedown');
  h.emit('document', 'mousemove', {clientX: 200, clientY: -1000});
  assert.equal(h.panel.style.top, '92px');
  h.emit('document', 'mouseup');
  h.emit('fixedWindowHandle', 'keydown', {key: 'ArrowUp'});
  assert.equal(h.panel.style.top, '92px');
});
