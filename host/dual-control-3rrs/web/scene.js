(function(){
  'use strict';
  const K=RrsKinematics,canvas=document.getElementById('scene'),ctx=canvas.getContext('2d');
  const view=p=>{const u=.82*p[0]-.57*p[1],depth=.57*p[0]+.82*p[1];return [u,-.43*depth-.9*p[2]];};
  let projection=null,drag=null,alignment=null;
  function draw(pose,angles,target,measured,mapping=PlateControl.DEFAULT_MAPPING,editorOpen=false){
    const rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
    canvas.width=rect.width*dpr;canvas.height=rect.height*dpr;ctx.scale(dpr,dpr);
    const w=rect.width,h=rect.height,available=w-(editorOpen&&window.innerWidth>1200?255:0);
    const scale=Math.min(available/.51,h/.35),center=[available*.5,h*.64];
    const point=p=>{const [x,y]=view(p);return [center[0]+x*scale,center[1]+y*scale];};
    const path=(points,color,width=1,dash=[],fill=null)=>{
      ctx.beginPath();points.forEach((p,i)=>{const q=point(p);i?ctx.lineTo(...q):ctx.moveTo(...q);});
      if(fill){ctx.closePath();ctx.fillStyle=fill;ctx.fill();}
      ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
    };
    ctx.clearRect(0,0,w,h);
    for(let i=-3;i<=3;i++){const a=i*.05;path([[a,-.18,0],[a,.18,0]],'#dfe2d9',.7);path([[-.18,a,0],[.18,a,0]],'#dfe2d9',.7);}
    const bases=[0,1,2].map(i=>{const a=i*2*Math.PI/3;return [K.geometry.r0*Math.cos(a),K.geometry.r0*Math.sin(a),0];});
    path([...bases,bases[0]],'#bcc9be',1.4,[], '#e4e8de80');
    const P=K.platform(pose);path([...P.joints,P.joints[0]],measured?'#243d3b':'#91a59a',2,[], '#cbdcd199');
    bases.forEach((B,i)=>{
      const a=i*2*Math.PI/3,t=angles[i]*K.RAD,r=K.geometry.r0+K.geometry.l1*Math.cos(t);
      const E=[r*Math.cos(a),r*Math.sin(a),-K.geometry.l1*Math.sin(t)];
      path([B,E], '#243d3b',4);path([E,P.joints[i]],'#648b7d',2.8);
      for(const p of [B,E,P.joints[i]]){const q=point(p);ctx.beginPath();ctx.arc(...q,3,0,Math.PI*2);ctx.fillStyle='#f5f3ec';ctx.fill();ctx.strokeStyle='#243d3b';ctx.lineWidth=1.4;ctx.stroke();}
      const q=point(B);ctx.fillStyle='#6c7c76';ctx.font='11px Consolas';ctx.fillText('M'+(i+1),q[0]+9,q[1]+17);
    });
    if(target){const T=K.platform(target);path([...T.joints,T.joints[0]],'#2671a8',1.5,[6,5]);}
    const O=P.center,origin=point(O);
    [['X',0],['Y',1],['Z',2]].forEach(([label,i])=>{
      const end=[...O];end[i]+=.050;path([O,end],'#91a49c',1,[3,3]);const q=point(end);
      ctx.fillStyle='#73877e';ctx.font='10px Consolas';ctx.fillText(label,q[0]+5,q[1]+12);
    });
    const axes=PlateControl.mappedAxes(mapping),handles={};
    [['pitch','Pitch +','#2671a8'],['roll','Roll +','#b46b3d']].forEach(([axis,label,color])=>{
      const direction=axes[axis],end=[O[0]+direction[0]*.088,O[1]+direction[1]*.088,O[2]];
      path([O,end],color,2.5);const q=point(end),angle=Math.atan2(q[1]-origin[1],q[0]-origin[0]);
      ctx.beginPath();ctx.moveTo(...q);
      ctx.lineTo(q[0]-10*Math.cos(angle-.4),q[1]-10*Math.sin(angle-.4));
      ctx.lineTo(q[0]-10*Math.cos(angle+.4),q[1]-10*Math.sin(angle+.4));ctx.closePath();ctx.fillStyle=color;ctx.fill();
      ctx.beginPath();ctx.arc(...q,8,0,Math.PI*2);ctx.strokeStyle=color;ctx.lineWidth=1;ctx.stroke();
      ctx.font='12px Consolas';ctx.fillText(label,q[0]+(q[0]>=origin[0]?12:-67),q[1]-10);
      handles[axis]=q;
    });
    projection={origin,scale,handles,mapping};
  }
  function bindAlignment(options){
    alignment=options;
    const local=e=>{const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
    const hit=q=>projection&&Object.keys(projection.handles).find(axis=>Math.hypot(...q.map((x,i)=>x-projection.handles[axis][i]))<19);
    canvas.addEventListener('pointerdown',e=>{
      if(e.button!==0||alignment.isLocked())return;
      drag=hit(local(e));if(!drag)return;
      canvas.setPointerCapture?.(e.pointerId);canvas.style.cursor='grabbing';e.preventDefault();
    });
    canvas.addEventListener('pointermove',e=>{
      if(!drag){canvas.style.cursor=!alignment.isLocked()&&hit(local(e))?'grab':'default';return;}
      if(alignment.isLocked()){drag=null;return;}
      const q=local(e),u=(q[0]-projection.origin[0])/projection.scale;
      const depth=-(q[1]-projection.origin[1])/projection.scale/.43;
      const x=.82*u+.57*depth,y=-.57*u+.82*depth;
      if(Math.hypot(x,y)<.01)return;
      const azimuth=Math.atan2(y,x)*K.DEG;
      alignment.onRotate(Math.round(PlateControl.rotationForAxis(drag,azimuth,projection.mapping)));
    });
    const end=e=>{drag=null;canvas.style.cursor='default';if(canvas.hasPointerCapture?.(e.pointerId))canvas.releasePointerCapture(e.pointerId);};
    canvas.addEventListener('pointerup',end);canvas.addEventListener('pointercancel',end);
  }
  window.PlateScene={draw,bindAlignment};
})();
