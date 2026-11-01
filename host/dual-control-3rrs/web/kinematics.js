(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.RrsKinematics = api;
})(globalThis, function () {
  'use strict';
  const RAD = Math.PI / 180, DEG = 1 / RAD;
  const geometry = Object.freeze({ r0: .126, rp: .137, l1: .060, l2: .102 });
  const homeAngles = Object.freeze([12.77, 12.77, 12.77]);
  const {r0, rp, l1, l2} = geometry;
  const homeZ = -l1 * Math.sin(homeAngles[0] * RAD) +
    Math.sqrt(l2*l2 - (rp-r0-l1*Math.cos(homeAngles[0]*RAD))**2);
  const homePose = Object.freeze([0, 0, homeZ]); // roll rad, pitch rad, height m
  const distance = (a,b) => Math.hypot(...a.map((x,i)=>x-b[i]));
  const near = (angle, reference) => angle + 360*Math.round((reference-angle)/360);

  // Direct transcription of the supplied strict 3D model, including Yp = -rp*r21.
  function platform(pose) {
    const [x,y,z] = pose, sx=Math.sin(x), cx=Math.cos(x), sy=Math.sin(y), cy=Math.cos(y);
    const yaw = Math.atan2(-sx*sy, cx+cy), sz=Math.sin(yaw), cz=Math.cos(yaw);
    const R = [[cy*cz,-cy*sz,sy], [cz*sx*sy+cx*sz,cx*cz-sx*sy*sz,-sx*cy],
      [sx*sz-cx*sy*cz,sx*cz+cx*sy*sz,cx*cy]];
    const center = [rp/2*(R[0][0]-R[1][1]), -rp*R[1][0], z];
    const joints = [0,1,2].map(i => {
      const a=i*2*Math.PI/3, c=Math.cos(a), s=Math.sin(a);
      return center.map((v,j)=>v+rp*(R[j][0]*c+R[j][1]*s));
    });
    return {R, center, joints, yaw};
  }

  function inverse(pose, previous=homeAngles) {
    if (pose.length!==3 || !pose.every(Number.isFinite) || pose[2]<=0)
      return {ok:false, reason:'平台高度必须在基座上方'};
    const result=platform(pose), angles=[], elbows=[], passive=[];
    for (let i=0;i<3;i++) {
      const a=i*2*Math.PI/3, c=Math.cos(a), s=Math.sin(a), A=result.joints[i];
      if (Math.abs(-A[0]*s+A[1]*c)>1e-8)
        return {ok:false, reason:`M${i+1} 切向闭环不满足`};
      const D=2*l1*c*(r0*c-A[0]), E=2*l1*A[2]*c*c;
      const F=A[0]**2-2*r0*A[0]*c+c*c*(r0*r0+l1*l1-l2*l2+A[2]*A[2]);
      const rho=Math.hypot(D,E), ratio=-F/rho;
      // No projection of an unreachable request into the workspace. Singular roots are rejected.
      if (!Number.isFinite(ratio) || Math.abs(ratio)>=1-1e-10)
        return {ok:false, reason:`M${i+1} 超出可达范围或接近支链奇异位形`};
      // The minus root is the installed assembly: at the calibrated home it equals +12.77 degrees.
      const theta=near((Math.atan2(E,D)-Math.acos(ratio))*DEG, previous[i]);
      if (Math.abs(theta)>180) return {ok:false, reason:`M${i+1} 超出角度指令编码范围`};
      const t=theta*RAD;
      const cp=(A[0]-c*(r0+l1*Math.cos(t)))/(l2*c), sp=-(A[2]+l1*Math.sin(t))/l2;
      const phi=Math.atan2(sp,cp);
      const B=[(r0+l1*Math.cos(t))*c,(r0+l1*Math.cos(t))*s,-l1*Math.sin(t)];
      const rebuilt=[B[0]+l2*Math.cos(phi)*c,B[1]+l2*Math.cos(phi)*s,B[2]-l2*Math.sin(phi)];
      if (distance(A,rebuilt)>1e-8 || Math.abs(D*Math.cos(t)+E*Math.sin(t)+F)>1e-10)
        return {ok:false, reason:`M${i+1} 三维闭环残差超限`};
      angles.push(theta); elbows.push(B); passive.push(phi);
    }
    return {...result, ok:true, pose:[...pose], angles, elbows, passive};
  }

  // Check the requested task-space path as well as its endpoint, preserving the installed branch.
  function reachableStep(from, to, previous) {
    const n=Math.max(1,Math.ceil(Math.max(Math.abs(to[0]-from[0])/RAD,
      Math.abs(to[1]-from[1])/RAD, Math.abs(to[2]-from[2])/.001)));
    let solution;
    for(let k=1;k<=n;k++) {
      solution=inverse(from.map((v,i)=>v+(to[i]-v)*k/n), previous);
      if(!solution.ok)return solution;
      previous=solution.angles;
    }
    return solution;
  }
  return {RAD,DEG,geometry,homeAngles,homeZ,homePose,platform,inverse,reachableStep};
});
