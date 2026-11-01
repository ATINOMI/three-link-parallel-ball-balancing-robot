'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const Plot=require('../drawing.js');

test('device time preserves 5ms sampling during USB bursts, unwraps tick overflow, and resets on reboot',()=>{
  const rows=[];
  for(const ms of [100,105,110,115])Plot.appendPlotSample(rows,{ms,value:ms});
  assert.deepEqual(rows.map(r=>r.t),[100,105,110,115]);
  assert.equal(Plot.sampling(rows).hz,200);
  const wrap=[];Plot.appendPlotSample(wrap,{ms:4294967290});Plot.appendPlotSample(wrap,{ms:4});
  assert.equal(wrap[1].t-wrap[0].t,10);
  Plot.appendPlotSample(rows,{ms:5});assert.equal(rows.length,1);assert.equal(rows[0].t,5);
  Plot.appendPlotSample(rows,{ms:60010});assert.equal(rows.length,1); // 60-second retention
});

test('visible-window autoscale excludes old transients, manual range remains exact, every sample is retained',()=>{
  const history=[{ms:0,t:0,value:1000},...Array.from({length:201},(_,i)=>({ms:5000+i*5,t:5000+i*5,value:i%2?.11:.09}))];
  const series=[{value:r=>r.value,color:'#000'}];
  const m=Plot.chartModel(history,series,[],{windowMs:1000,end:6000});
  assert.equal(m.rows.length,201);assert.ok(m.hi<.2);assert.ok(m.lo>0);
  assert.equal(m.rows[1],history[2]); // original samples, no interpolation or averaging
  const fixed=Plot.chartModel(history,series,[],{windowMs:1000,end:6000,yMin:.08,yMax:.12});
  assert.equal(fixed.lo,.08);assert.equal(fixed.hi,.12);
  assert.equal(Plot.nearestSample(m.rows,5006).ms,5005);
  assert.equal(Plot.nearestSample(m.rows,5009).ms,5010);
});

test('peak-to-peak, RMS and standard deviation expose small oscillation without smoothing it away',()=>{
  const rows=Array.from({length:200},(_,i)=>({t:i*5,v:i%2?.11:.09}));
  const s=Plot.statistics(rows,r=>r.v);
  assert.ok(Math.abs(s.p2p-.02)<1e-12);assert.ok(Math.abs(s.mean-.1)<1e-12);
  assert.ok(Math.abs(s.std-.01)<1e-12);assert.ok(Math.abs(s.rms-Math.sqrt(.0101))<1e-12);
  assert.equal(Plot.sampling(rows).median,5);
  const gap=[{t:0},{t:5},{t:10},{t:100}];assert.equal(Plot.sampling(gap).max,90);
});

test('canvas draws original sample segments with gap breaks, fixed scale and precise sample tooltip',()=>{
  global.window={devicePixelRatio:2};
  const operations=[];
  const ctx=new Proxy({measureText:s=>({width:s.length*7})},{get(o,k){if(k in o)return o[k];return (...args)=>operations.push([k,...args]);},set(o,k,v){o[k]=v;return true;}});
  const canvas={width:0,height:0,getBoundingClientRect:()=>({width:1000,height:420}),getContext:()=>ctx};
  const history=[0,5,10,100].map((t,i)=>({ms:t,t,v:i/1000}));
  const model=Plot.trend(canvas,history,[{label:'实测',value:r=>r.v,color:'#345e98'}],[],{windowMs:1000,end:100,yMin:-.01,yMax:.01,decimals:3,unit:'°/s',cursorRatio:.95});
  assert.equal(canvas.width,2000);assert.equal(canvas.height,840);assert.equal(model.rows.length,4);
  assert.ok(operations.some(o=>o[0]==='clip'));
  assert.ok(operations.some(o=>o[0]==='fillText' && String(o[1]).includes('设备')));
  assert.ok(operations.some(o=>o[0]==='fillText' && String(o[1]).includes('实测')));
  assert.equal(operations.some(o=>o[0]==='bezierCurveTo'||o[0]==='quadraticCurveTo'),false);
  delete global.window;
});
