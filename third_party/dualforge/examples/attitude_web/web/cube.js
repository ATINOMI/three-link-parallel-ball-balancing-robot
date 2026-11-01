(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.SketchCube=api;})(globalThis,function(){
  'use strict';
  const rotate=(q,p)=>{const [w,x,y,z]=q,[a,b,c]=p,t=[2*(y*c-z*b),2*(z*a-x*c),2*(x*b-y*a)];return [a+w*t[0]+y*t[2]-z*t[1],b+w*t[1]+z*t[0]-x*t[2],c+w*t[2]+x*t[1]-y*t[0]];};
  function interpolate(a,b,t){const sign=a.reduce((s,v,i)=>s+v*b[i],0)<0?-1:1,q=a.map((v,i)=>v*(1-t)+b[i]*sign*t),n=Math.hypot(...q);return q.map(v=>v/n);}
  const vertices=Array.from({length:8},(_,i)=>[i&1?1:-1,i&2?1:-1,i&4?1:-1]);
  const faces=[{v:[4,5,7,6],n:[0,0,1],color:'#edc69fb8',front:true},{v:[1,0,2,3],n:[0,0,-1],color:'#d6dcd2'},
    {v:[5,1,3,7],n:[1,0,0],color:'#c0d1d1ad'},{v:[0,4,6,2],n:[-1,0,0],color:'#c0d1d1ad'},
    {v:[6,7,3,2],n:[0,1,0],color:'#faf8ed'},{v:[0,1,5,4],n:[0,-1,0],color:'#dcdecf'}];
  function draw(canvas,q,ready=true){
    const rect=canvas.getBoundingClientRect(),w=rect.width,h=rect.height,dpr=Math.min(2,window.devicePixelRatio||1);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
    const size=Math.min(w*.24,h*.24),cx=w*.5,cy=h*.46;
    const view=p=>{const a=-.58,b=.36,x=Math.cos(a)*p[0]+Math.sin(a)*p[2],z=-Math.sin(a)*p[0]+Math.cos(a)*p[2];return [x,Math.cos(b)*p[1]-Math.sin(b)*z,Math.sin(b)*p[1]+Math.cos(b)*z];};
    const project=p=>{const v=view(rotate(q,p)),scale=size*7/(7-v[2]);return [cx+v[0]*scale,cy-v[1]*scale,v[2]];};
    const points=vertices.map(project);c.globalAlpha=ready?1:.38;
    c.fillStyle='#9aab9920';c.beginPath();c.ellipse(cx,cy+size*1.6,size*1.35,size*.18,-.04,0,Math.PI*2);c.fill();
    function polygon(ids){c.beginPath();ids.forEach((i,n)=>n?c.lineTo(points[i][0],points[i][1]):c.moveTo(points[i][0],points[i][1]));c.closePath();}
    function sketchLine(a,b,seed,width=1.5,color='#334246'){
      const noise=Math.sin(seed*39.7)*1.2,mx=(a[0]+b[0])/2,my=(a[1]+b[1])/2;c.strokeStyle=color;c.lineWidth=width;c.lineCap='round';
      c.beginPath();c.moveTo(a[0]-.5,a[1]+.3);c.quadraticCurveTo(mx+noise,my-noise,b[0]+.3,b[1]-.3);c.stroke();
      c.globalAlpha*=.24;c.beginPath();c.moveTo(a[0]+1,a[1]-.7);c.lineTo(b[0]-.7,b[1]+1);c.stroke();c.globalAlpha/= .24;
    }
    faces.filter(f=>view(rotate(q,f.n))[2]>.002).sort((a,b)=>a.v.reduce((s,i)=>s+points[i][2],0)-b.v.reduce((s,i)=>s+points[i][2],0)).forEach((f,index)=>{
      polygon(f.v);c.fillStyle=f.color;c.fill();
      if(f.front){c.save();polygon(f.v);c.clip();c.strokeStyle='#9f765b30';c.lineWidth=.7;for(let x=cx-size*3;x<cx+size*3;x+=13){c.beginPath();c.moveTo(x,cy+size*2);c.lineTo(x+size*2,cy-size*2);c.stroke();}c.restore();}
      f.v.forEach((v,i)=>sketchLine(points[v],points[f.v[(i+1)%4]],v+index*3));
      if(f.front){const lines=[[[-.26,-.33,1.007],[-.26,.33,1.007]],[[-.26,.33,1.007],[.28,.33,1.007]],[[-.26,.03,1.007],[.15,.03,1.007]]];lines.forEach((line,i)=>sketchLine(project(line[0]),project(line[1]),19+i,2.4,'#846044'));}
    });
    // Small body-axis sketch; its rotation uses exactly the cube quaternion.
    const origin=[w-64,h-49],axisScale=22;
    [[1,0,0,'X','#7e9ca6'],[0,1,0,'Y','#819475'],[0,0,1,'Z','#c09370']].forEach(([x,y,z,label,color])=>{
      const p=view(rotate(q,[x,y,z])),end=[origin[0]+p[0]*axisScale,origin[1]-p[1]*axisScale];sketchLine(origin,end,7,1.2,color);c.font='9px Consolas';c.fillStyle=color;c.fillText(label,end[0]+3,end[1]+3);
    });
    c.globalAlpha=1;
  }
  return {rotate,interpolate,draw};
});
