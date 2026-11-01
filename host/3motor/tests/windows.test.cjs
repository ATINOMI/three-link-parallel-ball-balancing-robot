'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const {boundedSize, install} = require('../windows.js');

function harness() {
  const nodes = new Map(), events = {}, windowEvents = {};
  let drawings = 0, dragPredicate;
  const chart = {paused:true, snapshot:[{t:123}], expanded:false};
  function node(id) {
    if (!nodes.has(id)) {
      const classes = new Set();
      nodes.set(id, {id, style: {}, attrs: {}, events: {}, hidden:false,
        classList:{contains:key=>classes.has(key),add:(...keys)=>keys.forEach(key=>classes.add(key)),remove:(...keys)=>keys.forEach(key=>classes.delete(key))},
        setAttribute(key,value){this.attrs[key]=value;}, removeAttribute(key){delete this.attrs[key];}, focus(){},
        addEventListener(type,fn){(this.events[type] ||= []).push(fn);},
        getBoundingClientRect(){return {left:parseFloat(this.style.left)||100,top:parseFloat(this.style.top)||100,width:parseFloat(this.style.width)||440,height:parseFloat(this.style.height)||400};}});
    }
    return nodes.get(id);
  }
  const doc={getElementById:node,querySelector:()=>({getBoundingClientRect:()=>({bottom:58})}),documentElement:{style:{setProperty(){}}},
    addEventListener(type,fn){(events[type] ||= []).push(fn);}};
  const win={innerWidth:1366,innerHeight:768,addEventListener(type,fn){(windowEvents[type] ||= []).push(fn);}};
  install(doc,win,()=>drawings++,(...args)=>{dragPredicate=args[5];},chart);
  function emit(target,type,props={}) {
    const e={button:0,pointerId:1,clientX:540,clientY:500,preventDefault(){},stopPropagation(){},...props};
    for(const fn of (target==='document'?events[type]:target==='window'?windowEvents[type]:node(target).events[type])||[])fn(e);
  }
  return {node,emit,win,chart,canDrag:()=>dragPredicate(),drawings:()=>drawings};
}

test('resize bounds preserve reachable grips and adapt minimum sizes to small viewports',()=>{
  assert.deepEqual(boundedSize(9999,9999,{left:100,top:100},{width:1366,height:768,header:58},{width:360,height:240}),{width:1254,height:656});
  assert.deepEqual(boundedSize(1,1,{left:8,top:100},{width:320,height:400,header:58},{width:360,height:240}),{width:300,height:240});
});

test('all six windows independently stretch width and height, with document-level dragging',()=>{
  const h=harness();h.node('responseOpenBtn').onclick();
  for(const id of ['fixedWindow','speedParamsWindow','speedTrackWindow','positionParamsWindow','positionTrackWindow','responsePanel']) {
    const panel=h.node(id),before=panel.getBoundingClientRect();
    h.emit(id+'Resize','pointerdown');
    h.emit('document','pointermove',{pointerId:99,clientX:640});
    assert.equal(panel.getBoundingClientRect().width,before.width);
    h.emit('document','pointermove',{clientX:620});
    assert.equal(panel.getBoundingClientRect().width,before.width+80);
    assert.equal(panel.getBoundingClientRect().height,before.height);
    h.emit('document','pointermove',{clientX:620,clientY:450});
    assert.equal(panel.getBoundingClientRect().height,before.height-50);
    h.emit('document','pointerup');
    h.emit('document','pointermove',{clientX:900});
    assert.equal(panel.getBoundingClientRect().width,before.width+80);
    h.emit(id+'Resize','keydown',{key:'ArrowRight',shiftKey:true});
    assert.equal(panel.getBoundingClientRect().width,before.width+88);
  }
  assert.ok(h.drawings()>4);
});

test('response floats from both entries, returns inline and remembers floating geometry without clearing frozen samples',()=>{
  const h=harness(),panel=h.node('responsePanel'),snapshot=h.chart.snapshot;
  assert.equal(h.canDrag(),false);
  h.emit('responsePanelResize','keydown',{key:'ArrowRight'});
  assert.equal(panel.style.width,undefined);
  h.node('responseFloatBtn').onclick();assert.equal(h.canDrag(),true);
  panel.style.width='700px';panel.style.height='500px';
  h.node('responseCloseBtn').onclick();
  assert.equal(h.canDrag(),false);assert.equal(panel.hidden,false);assert.equal(panel.style.width,undefined);
  assert.equal(h.chart.snapshot,snapshot);assert.equal(h.chart.paused,true);
  h.node('responseOpenBtn').onclick();
  assert.equal(panel.style.width,'700px');assert.equal(panel.style.height,'500px');
  assert.equal(panel.attrs.role,'dialog');assert.equal(h.node('responseCloseBtn').hidden,false);
});

test('maximized charts ignore resize, and pointer cancellation stops resizing',()=>{
  const h=harness(),panel=h.node('speedTrackWindow');
  panel.classList.add('plot-maximized');
  h.emit('speedTrackWindowResize','keydown',{key:'ArrowRight'});assert.equal(panel.style.width,undefined);
  panel.classList.remove('plot-maximized');
  h.emit('speedTrackWindowResize','pointerdown');h.emit('document','pointercancel');
  h.emit('document','pointermove',{clientX:700});assert.equal(panel.style.width,undefined);
});
