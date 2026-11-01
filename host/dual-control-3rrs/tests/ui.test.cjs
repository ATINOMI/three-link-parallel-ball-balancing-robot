const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const K=require('../web/kinematics.js');
function boot(storage=new Map()){
  const web=path.join(__dirname,'../web'),html=fs.readFileSync(path.join(web,'index.html'),'utf8');
  const elements=Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{textContent:'',disabled:false,className:'',value:'',checked:false,style:{},open:false}]));
  const canvasEvents={},labels=[];
  // Use a real property bag for fillText, with no-op drawing methods.
  elements.scene.getContext=()=>new Proxy({fillText:(text,x,y)=>labels.push({text,x,y})},{get:(obj,key)=>obj[key]||(()=>{}),set:(obj,key,value)=>(obj[key]=value,true)});
  elements.scene.getBoundingClientRect=()=>({width:850,height:430,left:0,top:0});
  elements.scene.addEventListener=(name,fn)=>canvasEvents[name]=fn;
  const listeners={},timers=[],sent=[],instances=[];
  const document={getElementById:id=>{assert(elements[id],`Missing DOM #${id}`);return elements[id];},hidden:false,addEventListener:(name,fn)=>listeners[name]=fn};
  const sandbox={document,window:null,globalThis:null,navigator:{},performance:{now:()=>0},AbortSignal,TextEncoder,TextDecoder,
    localStorage:{getItem:key=>storage.get(key)||null,setItem:(key,value)=>storage.set(key,value)},
    setInterval:(fn,ms)=>timers.push({fn,ms}),requestAnimationFrame:()=>{},fetch:async()=>({ok:true,json:async()=>({ready:false,connected:false,session:0,circle_seq:0})})};
  sandbox.window=sandbox;sandbox.globalThis=sandbox;sandbox.devicePixelRatio=1;sandbox.innerWidth=1400;sandbox.addEventListener=(name,fn)=>listeners[name]=fn;
  const ctx=vm.createContext(sandbox);
  for(const [,file]of html.matchAll(/<script src="\/([^"]+)"/g)){
    if(file==='app.js'){
      const Base=ctx.PlateControl.Control;
      ctx.PlateControl.Control=class extends Base{constructor(...args){super(...args);instances.push(this);}};
      ctx.MotorSerial.SerialLink.prototype.send=function(line){sent.push(line);return Promise.resolve(true);};
    }
    vm.runInContext(fs.readFileSync(path.join(web,file),'utf8'),ctx,{filename:file});
  }
  return {elements,listeners,timers,ctx,storage,sent,control:instances[0],canvasEvents,labels};
}
test('the shipped HTML initializes and binds Q emergency stop without hardware',async()=>{
  const f=boot();await new Promise(resolve=>setImmediate(resolve));
  assert.equal(f.elements.start.disabled,true);assert.equal(f.elements.home.disabled,true);assert.equal(f.timers.length,2);assert.deepEqual(f.sent,[]);
  f.listeners.keydown({code:'KeyQ',key:'q',preventDefault(){}});assert.match(f.elements.message.textContent,/Q 键急停/);
});
test('axis presets, swap and independent signs save/reload without sending any motion',()=>{
  const f=boot();assert.equal(f.elements['axis-angle'].value,'150');
  f.elements['axis-preset'].value='0';f.elements['axis-preset'].onchange();
  f.elements['axis-swap'].checked=true;f.elements['axis-swap'].onchange();
  f.elements['axis-pitch-sign'].value='-1';f.elements['axis-pitch-sign'].onchange();
  assert.equal(f.control.mapping.rotationDeg,0);assert.equal(f.control.mapping.swap,true);assert.equal(f.control.mapping.pitchSign,-1);
  assert.deepEqual(f.sent,[]);const restored=boot(f.storage);
  assert.equal(restored.control.mapping.pitchSign,-1);assert.equal(restored.elements['axis-swap'].checked,true);
  f.elements['axis-reset'].onclick();assert.equal(f.control.mapping.rotationDeg,150);assert.equal(f.control.mapping.swap,false);
});
test('active control locks both form and drag mapping; invalid stored settings use defaults',()=>{
  const f=boot(new Map([['3rrs-axis-mapping-v1','{"rotationDeg":999}']]));assert.equal(f.control.mapping.rotationDeg,150);
  f.control.acceptHand({session:1,input_version:2,ready:true,calibrating:false,demo:false,angles:[0,0,0],circle_seq:0,dpad_up_seq:0,dpad_down_seq:0});
  f.control.acceptFrame({version:'V10',mode:1,fresh:7,pose:K.homePose,angles:K.homeAngles});f.control.start();
  f.elements['axis-angle'].value='0';f.elements['axis-angle'].onchange();
  assert.equal(f.control.mapping.rotationDeg,150);f.timers.find(t=>t.ms===50).fn();
  assert.equal(f.elements['pose-sensitivity'].disabled,true);assert.equal(f.elements['axis-fields'].disabled,true);assert.equal(f.elements['axis-pitch-sign'].disabled,true);
});
test('dragging a plotted positive axis updates the mapping and persists it without motor commands',()=>{
  const f=boot(),ctx=f.ctx,rect=f.elements.scene.getBoundingClientRect();
  ctx.PlateScene.draw(K.homePose,K.homeAngles,null,false,f.control.mapping,false);
  const label=f.labels.filter(x=>x.text==='Pitch +').at(-1);
  const origin=[rect.width*.5,rect.height*.64-.9*K.homeZ*Math.min(rect.width/.51,rect.height/.35)];
  const axis=ctx.PlateControl.mappedAxes(f.control.mapping).pitch;
  const scale=Math.min(rect.width/.51,rect.height/.35);
  const end=[origin[0]+(.82*axis[0]-.57*axis[1])*.088*scale,origin[1]-.43*(.57*axis[0]+.82*axis[1])*.088*scale];
  assert(label);f.canvasEvents.pointerdown({button:0,pointerId:1,clientX:end[0],clientY:end[1],preventDefault(){}});
  f.canvasEvents.pointermove({clientX:origin[0]+.82*.088*scale,clientY:origin[1]-.43*.57*.088*scale});
  f.canvasEvents.pointerup({pointerId:1});assert.equal(f.control.mapping.rotationDeg,-90);assert.deepEqual(f.sent,[]);
});

test('legacy saved axes survive sensitivity upgrade, edits, reset and reload unchanged',()=>{
  const axes={rotationDeg:43.25,swap:true,pitchSign:-1,rollSign:1};
  const storage=new Map([['3rrs-axis-mapping-v1',JSON.stringify(axes)]]),f=boot(storage);
  for(const [key,value]of Object.entries(axes))assert.equal(f.control.mapping[key],value);
  assert.equal(f.elements['pose-sensitivity'].value,'1');
  // Reading an old configuration does not overwrite its stored contents.
  assert.equal(storage.get('3rrs-axis-mapping-v1'),JSON.stringify(axes));
  f.elements['pose-sensitivity'].value='0.4';f.elements['pose-sensitivity'].onchange();
  for(const [key,value]of Object.entries(axes))assert.equal(f.control.mapping[key],value);
  assert.equal(f.control.mapping.sensitivity,.4);assert.deepEqual(f.sent,[]);
  const restored=boot(storage);assert.equal(restored.control.mapping.sensitivity,.4);
  for(const [key,value]of Object.entries(axes))assert.equal(restored.control.mapping[key],value);
  restored.elements['axis-reset'].onclick();assert.equal(restored.control.mapping.sensitivity,.4);
});
test('invalid sensitivity preserves both current gain and axis settings',()=>{
  const f=boot();f.elements['pose-sensitivity'].value='-1';f.elements['pose-sensitivity'].onchange();
  assert.equal(f.control.mapping.sensitivity,1);assert.equal(f.control.mapping.rotationDeg,150);
  assert.match(f.elements['axis-save-state'].textContent,/0～10/);assert.deepEqual(f.sent,[]);
});
