const {test}=require('node:test');
const assert=require('node:assert/strict');
const K=require('../web/kinematics.js');
const {Control,mapTilt,stepHeight,M3_AZIMUTH}=require('../web/control.js');
const {parseTelemetry,LineReader}=require('../web/core.js');
function fixture(){
  let now=0;const commands=[],c=new Control({send:s=>commands.push(s),now:()=>now});
  const hand={session:1,ready:true,connected:true,calibrating:false,demo:false,angles:[0,0,0],left_stick:[0,0],circle_seq:0,input_version:2,dpad_up_seq:0,dpad_down_seq:0};
  const frame={version:'V10',mode:1,fresh:7,pose:[...K.homePose],angles:[...K.homeAngles],angleTarget:[...K.homeAngles],angleSettling:{toleranceDeg:.5,holdMs:20,state:[0,0,0]}};
  c.acceptHand(hand);c.acceptFrame(frame);
  function step(ms=50){now+=ms;c.acceptHand(hand);c.acceptFrame(frame);c.tick();}
  return {c,hand,frame,commands,step,advance:ms=>now+=ms};
}
test('home geometry closes at all three +12.77 degree angles',()=>{
  const s=K.inverse(K.homePose);assert(s.ok);assert(Math.abs(K.homeZ-.0769942392749089)<1e-12);
  s.angles.forEach(t=>assert(Math.abs(t-12.77)<1e-10));
});
test('independent 3D distance and tangential closure across workspace samples',()=>{
  let count=0;
  for(let x=-.16;x<=.16;x+=.04)for(let y=-.16;y<=.16;y+=.04)for(let z=.065;z<.125;z+=.01){
    const s=K.inverse([x,y,z]);if(!s.ok)continue;count++;
    s.joints.forEach((A,i)=>{const a=i*2*Math.PI/3,t=s.angles[i]*K.RAD;
      const B=[(.126+.060*Math.cos(t))*Math.cos(a),(.126+.060*Math.cos(t))*Math.sin(a),-.060*Math.sin(t)];
      assert(Math.abs(Math.hypot(...A.map((v,j)=>v-B[j]))-.102)<1e-9);
      assert(Math.abs(-A[0]*Math.sin(a)+A[1]*Math.cos(a))<1e-9);
    });
  }assert(count>200);
});
test('higher platform yields decreasing motor angles; unreachable requests are rejected',()=>{
  const s=K.inverse([0,0,K.homeZ+.01]);assert(s.ok);assert(s.angles.every(t=>t<12.77));
  assert.equal(K.inverse([0,0,.3]).ok,false);assert.equal(K.inverse([0,0,0]).ok,false);
  assert.equal(K.inverse([1,1,.08]).ok,false);
});
test('dependent yaw/negative Yp remain part of the model',()=>{
  const s=K.inverse([.1,.08,.09]);assert(s.ok);
  assert(Math.abs(s.yaw)>0);assert.equal(s.center[1],-.137*s.R[1][0]);
});
test('connecting and displaying alone never sends a motion command',()=>{
  const f=fixture();f.step();assert.deepEqual(f.commands,[]);
});
test('V9 cannot arm control or home',()=>{
  const f=fixture();f.frame.version='V9';assert.throws(()=>f.c.start(),/V10/);assert.throws(()=>f.c.home(),/V10/);
});
test('start is bumpless and ignores controller yaw',()=>{
  const f=fixture();f.c.start();const before=[...f.c.targets];f.hand.angles[2]=150;f.step();
  assert.deepEqual(f.c.targets,before);assert.equal(f.c.feedback,'normal');
});
test('D-pad press steps height by five equivalent motor degrees; held button and stick cannot repeat',()=>{
  const f=fixture();f.c.start();f.frame.mode=6;f.hand.dpad_up_seq++;f.step();
  const raised=f.c.pose[2];assert(raised>K.homeZ);
  f.c.targets.forEach(t=>assert(Math.abs(t-7.77)<1e-9));
  f.hand.left_stick[1]=-1;for(let i=0;i<15;i++)f.step();assert.equal(f.c.pose[2],raised);
  f.hand.dpad_down_seq++;f.step();assert(Math.abs(f.c.pose[2]-K.homeZ)<1e-12);
  f.hand.dpad_down_seq++;f.step();assert(f.c.pose[2]<K.homeZ);
  f.c.targets.forEach(t=>assert(Math.abs(t-17.77)<1e-9));
});
test('finite pure pitch rotates around the center-to-M3 radial axis',()=>{
  const axis=[Math.cos(M3_AZIMUTH),Math.sin(M3_AZIMUTH),0],p=12*K.RAD;
  const tilt=mapTilt([0,0],0,p),R=K.platform([...tilt,K.homeZ]).R;
  const rotated=R.map(row=>row.reduce((v,x,i)=>v+x*axis[i],0));
  rotated.forEach((v,i)=>assert(Math.abs(v-axis[i])<1e-12));
  assert(Math.abs(R[0][2]-axis[1]*Math.sin(p))<1e-12);
  assert(Math.abs(R[1][2]+axis[0]*Math.sin(p))<1e-12);
  const base=[.08,-.06];mapTilt(base,0,0).forEach((v,i)=>assert(Math.abs(v-base[i])<1e-12));
});
test('height step preserves the mapped tilt and works on a non-flat platform',()=>{
  const f=fixture();f.c.start();f.frame.mode=6;f.hand.angles=[3,4,0];f.step();const tilt=f.c.pose.slice(0,2);
  f.hand.dpad_up_seq++;f.step();assert.deepEqual(f.c.pose.slice(0,2),tilt);
  assert(Math.abs(f.c.pose[2]-stepHeight(K.homeZ,1).z)<1e-12);
});
test('D-pad events before start, hidden or from an old service cannot arm motion',()=>{
  const f=fixture();f.hand.dpad_up_seq++;f.step();assert.deepEqual(f.commands,[]);
  f.c.start();f.frame.mode=6;f.step();assert.equal(f.c.pose[2],K.homeZ);
  f.c.inputEnabled=false;f.hand.dpad_up_seq++;f.step();assert.equal(f.c.pose[2],K.homeZ);
  const old=fixture();delete old.hand.input_version;assert.throws(()=>old.c.start(),/新版/);
});
test('unreachable height steps hold all three targets without accumulating further beyond the edge',()=>{
  const f=fixture();f.c.start();f.frame.mode=6;
  for(let i=0;i<40&&!f.c.invalid;i++){f.hand.dpad_up_seq++;f.step();}
  assert(f.c.invalid);const before=[...f.c.targets],height=f.c.pose[2];
  f.hand.dpad_up_seq++;f.step();assert(f.c.invalid);assert.deepEqual(f.c.targets,before);assert.equal(f.c.pose[2],height);
  f.hand.dpad_down_seq++;f.step();assert(!f.c.invalid);assert(f.c.pose[2]<height);
});
test('unreachable tilt holds complete last valid motor target; reachable tilt clears alarm',()=>{
  const f=fixture();f.c.start();f.frame.mode=6;f.hand.angles[0]=4;f.step();const valid=[...f.c.targets];
  f.hand.angles[0]=75;f.step();assert(f.c.invalid);assert.equal(f.c.feedback,'invalid');assert.deepEqual(f.c.targets,valid);
  assert.equal(f.commands.at(-1),'A,'+valid.map(v=>Math.round(v*1000)).join(','));
  f.hand.angles[0]=0;f.step();assert(!f.c.invalid);assert.equal(f.c.feedback,'normal');
});
test('O resets all targets and waits for actual firmware stable arrival before recentering',()=>{
  const f=fixture();f.c.start();f.frame.mode=6;f.hand.angles[0]=3;f.step();f.hand.circle_seq++;f.step();
  assert.equal(f.c.state,'homing');assert.deepEqual(f.c.targets,K.homeAngles);
  f.hand.angles[0]=10;f.frame.angleTarget=[-10,-10,-10];f.frame.angleSettling.state=[2,2,2];f.step();
  assert.equal(f.c.state,'homing');
  f.frame.angleTarget=[...K.homeAngles];f.step();assert.equal(f.c.state,'running');
  assert(Math.abs(f.c.pose[0])<1e-10);f.step();assert(Math.abs(f.c.pose[0])<1e-10);
});
test('O from idle homes and holds without automatically enabling following',()=>{
  const f=fixture();f.hand.circle_seq++;f.step();assert.equal(f.c.state,'homing');
  f.frame.mode=6;f.frame.angleSettling.state=[2,2,2];f.step();assert.equal(f.c.state,'holding');
  f.hand.angles[0]=30;f.step();assert.deepEqual(f.c.targets,K.homeAngles);
});
test('controller disconnect and reconnect stops and requires explicit restart',()=>{
  const f=fixture();f.c.start();f.hand.session=2;f.step();assert.equal(f.c.state,'idle');assert.equal(f.commands.at(-1),'S');
  const n=f.commands.length;f.step();assert.equal(f.commands.length,n);
});
test('hidden page and stale button events cannot initiate homing',()=>{
  const f=fixture();f.c.inputEnabled=false;f.hand.circle_seq++;f.step();assert.equal(f.c.state,'idle');
  f.c.inputEnabled=true;f.advance(300);f.hand.circle_seq++;f.c.acceptHand(f.hand);assert.equal(f.c.state,'idle');
  f.hand.circle_seq++;f.step();assert.equal(f.c.state,'homing');
});
test('telemetry timeout, feedback loss, mode exit and scheduling stalls clear current',()=>{
  for(const failure of ['timeout','fresh','mode','stall']){
    const f=fixture();f.c.start();f.frame.mode=6;f.step();
    if(failure==='timeout'){f.advance(501);f.c.acceptHand(f.hand);f.c.tick();}
    if(failure==='fresh'){f.frame.fresh=0;f.step();}
    if(failure==='mode'){f.frame.mode=0;f.step();}
    if(failure==='stall')f.step(250);
    assert.equal(f.c.state,'idle',failure);assert.equal(f.commands.at(-1),'S',failure);
  }
});
test('no mode acknowledgement causes stop after one second',()=>{
  const f=fixture();f.c.start();for(let i=0;i<21;i++)f.step();assert.equal(f.c.state,'idle');assert.equal(f.commands.at(-1),'S');
});
test('V10 parses fragmented telemetry with the unchanged V9 field layout',()=>{
  const fields=Array(104).fill(0);fields[1]=6;fields[4]=7;fields[99]=500;fields[100]=20;
  const line='V10,'+fields.join(',');const frame=parseTelemetry(line);
  assert.equal(frame.version,'V10');assert.equal(frame.angleSettling.toleranceDeg,.5);
  const frames=[],reader=new LineReader(s=>frames.push(parseTelemetry(s)));reader.push(line.slice(0,35));reader.push(line.slice(35)+'\r\n');
  assert.equal(frames.length,1);assert.equal(frames[0].version,'V10');
});

test('mapping presets, swapping and signs affect actual closed-loop targets; edits never arm movement',()=>{
  const f=fixture(),normal={rotationDeg:0,swap:false,rollSign:1,pitchSign:1};
  f.c.setMapping(normal);assert.deepEqual(f.commands,[]);f.c.start();f.frame.mode=6;
  f.hand.angles=[0,5,0];f.step();assert(Math.abs(f.c.pose[0])<1e-12);assert(Math.abs(f.c.pose[1]-5*K.RAD)<1e-12);
  assert.throws(()=>f.c.setMapping({...normal,pitchSign:-1}),/停止/);
  f.c.stop();f.c.setMapping({...normal,swap:true,pitchSign:-1});f.hand.angles=[0,0,0];f.c.start();
  f.hand.angles=[0,5,0];f.step();assert(Math.abs(f.c.pose[0]+5*K.RAD)<1e-12);assert(Math.abs(f.c.pose[1])<1e-12);
});
test('mapping axes and drag inverse agree for all swaps and polarity combinations',()=>{
  const {mappedAxes,rotationForAxis}=require('../web/control.js');
  for(const rotationDeg of [-140,0,30,150])for(const swap of [false,true])for(const pitchSign of [-1,1])for(const rollSign of [-1,1]){
    const m={rotationDeg,swap,pitchSign,rollSign},axes=mappedAxes(m);
    assert(Math.abs(axes.roll[0]*axes.pitch[0]+axes.roll[1]*axes.pitch[1])<1e-12);
    for(const axis of ['roll','pitch']){
      const angle=Math.atan2(axes[axis][1],axes[axis][0])*K.DEG;
      assert(Math.abs(rotationForAxis(axis,angle,m)-rotationDeg)<1e-10);
      const r=axis==='roll'?.07:0,p=axis==='pitch'?.07:0,tilt=mapTilt([0,0],r,p,m),R=K.platform([...tilt,K.homeZ]).R;
      assert(Math.abs(R[0][2]-axes[axis][1]*Math.sin(.07))<1e-12);
      assert(Math.abs(R[1][2]+axes[axis][0]*Math.sin(.07))<1e-12);
    }
  }
});

test('sensitivity scales relative attitude before finite axis mapping without changing height step',()=>{
  const f=fixture();f.c.setMapping({rotationDeg:0,swap:false,rollSign:1,pitchSign:1,sensitivity:.5});
  f.c.start();f.frame.mode=6;f.hand.angles=[0,8,0];f.step();
  assert(Math.abs(f.c.pose[1]-4*K.RAD)<1e-12);
  f.hand.dpad_up_seq++;f.step();assert(Math.abs(f.c.pose[2]-stepHeight(K.homeZ,1).z)<1e-12);
  const base=[.04,-.03],legacy={rotationDeg:43,swap:true,rollSign:-1,pitchSign:1};
  for(const gain of [0,.25,1,2]){
    const actual=mapTilt(base,.04,-.07,{...legacy,sensitivity:gain}),expected=mapTilt(base,.04*gain,-.07*gain,legacy);
    actual.forEach((v,i)=>assert(Math.abs(v-expected[i])<1e-12));
  }
});
test('zero sensitivity holds base attitude while D-pad still controls height',()=>{
  const f=fixture();f.c.setMapping({...f.c.mapping,sensitivity:0});f.c.start();f.frame.mode=6;
  f.hand.angles=[20,-15,80];f.hand.dpad_up_seq++;f.step();
  assert(Math.abs(f.c.pose[0])<1e-12&&Math.abs(f.c.pose[1])<1e-12);assert(f.c.pose[2]>K.homeZ);
});

test('V11 min-speed telemetry remains usable by gamepad control without resetting saved input settings',()=>{
  const f=fixture();f.frame.version='V11';f.c.setMapping({...f.c.mapping,rotationDeg:42,swap:true,pitchSign:-1,sensitivity:.4});
  f.c.start();f.frame.mode=6;f.step();assert.equal(f.c.state,'running');assert.equal(f.c.mapping.rotationDeg,42);assert.equal(f.c.mapping.sensitivity,.4);
  const fields=Array(107).fill(0);fields[1]=6;fields[4]=7;fields[99]=500;fields[100]=20;fields[104]=1000;fields[105]=2000;fields[106]=3000;
  const parsed=parseTelemetry('V11,'+fields.join(','));assert.deepEqual(parsed.anglePid.map(p=>p.minSpeed),[1,2,3]);
});
