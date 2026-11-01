const test=require('node:test'),assert=require('node:assert/strict');
const Cube=require('./web/cube.js');
test('SDL quaternion axes rotate the cube without mirroring',()=>{
  const s=Math.sqrt(.5),near=(a,b)=>a.every((v,i)=>Math.abs(v-b[i])<1e-10);
  assert.ok(near(Cube.rotate([s,s,0,0],[0,1,0]),[0,0,1]));
  assert.ok(near(Cube.rotate([s,0,s,0],[0,0,1]),[1,0,0]));
  assert.ok(near(Cube.rotate([s,0,0,s],[1,0,0]),[0,1,0]));
});
test('display interpolation remains unit length and handles equivalent quaternion signs',()=>{
  assert.deepEqual(Cube.interpolate([1,0,0,0],[-1,0,0,0],.5),[1,0,0,0]);
  const q=Cube.interpolate([1,0,0,0],[.5,.5,.5,.5],.3);assert.ok(Math.abs(Math.hypot(...q)-1)<1e-12);
});
test('hand-drawn cube renders all orientations and waiting state with a deterministic canvas',()=>{
  global.window={devicePixelRatio:1.5};let strokes=0;
  const context=new Proxy({stroke(){strokes++;}},{get:(t,k)=>k in t?t[k]:()=>{}});
  const canvas={width:0,height:0,getBoundingClientRect:()=>({width:700,height:408}),getContext:()=>context};
  for(const q of [[1,0,0,0],[0,1,0,0],[.5,.5,.5,.5],[Math.sqrt(.5),0,Math.sqrt(.5),0]])Cube.draw(canvas,q,true);
  Cube.draw(canvas,[1,0,0,0],false);assert.ok(strokes>60);assert.equal(canvas.width,1050);assert.equal(canvas.height,612);
});
