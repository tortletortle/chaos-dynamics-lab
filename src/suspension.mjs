// V0.4 reduced-order longitudinal/vertical suspension model with wheel spin and energy audit.
export function createSuspensionLab(){
  // Planar, linearized chassis pitch. Axles share x but have independent vertical
  // displacement and angular speed. No lateral, linkage or rigid collision solver.
  const G=9.81,Kt=42000,Ct=420,rest=.30,offset=.12,slipScale=.55;
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  const base={n:2,span:1.7,radius:.38,gear:4.5,stiffness:36000,damping:2300,com:0};
  function rng(seed){let s=seed>>>0;return ()=>{s=(s+0x6d2b79f5)>>>0;let t=Math.imul(s^(s>>>15),1|s);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
  function terrain(seed,kind="rough"){
    if(!["flat","rough","ridge","washboard","low-grip"].includes(kind))throw Error("invalid terrain");
    const r=rng(seed),p=r()*2*Math.PI,a=.85+.3*r(),mu=kind==="low-grip"?.17:kind==="flat"?.8:.52+.22*r();
    function h(x){const z=x-3;if(z<=0||kind==="flat"||kind==="low-grip")return 0;const w=clamp(z/2,0,1),fade=w*w*(3-2*w);
      if(kind==="ridge")return fade*a*(.65*Math.exp(-((z-4)**2)/3)+.48*Math.exp(-((z-11)**2)/2));
      if(kind==="washboard")return fade*a*(.13*Math.sin(3.1*z+p)+.08*Math.sin(5*z));
      return fade*a*(.26*Math.sin(.9*z+p)+.13*Math.sin(2.2*z+p/2));}
    return {h,mu,slope:x=>(h(x+.0001)-h(x-.0001))/.0002};
  }
  function model(g,seed,{kind="rough",drive=true}={}){
    if(!g||!Number.isInteger(g.n)||g.n<2||g.n>4||!["span","radius","gear","stiffness","damping","com"].every(k=>Number.isFinite(g[k])))throw Error("invalid genome");
    if(g.span<1.2||g.span>3.4||g.radius<.22||g.radius>.55||g.gear<2||g.gear>8||g.stiffness<18000||g.stiffness>80000||g.damping<500||g.damping>5000||Math.abs(g.com)>.15||g.span/(g.n-1)<2*g.radius+.05-1e-10)throw Error("geometry outside V0.4 envelope");
    const {h,mu,slope}=terrain(seed,kind),n=g.n,M=86+12*g.span,mw=9+12*g.radius*g.radius,J=.5*mw*g.radius*g.radius,I=M*(g.span*g.span+.25)/12,Mx=M+n*mw,ks=g.stiffness/n,cs=g.damping/n;
    const lx=Array.from({length:n},(_,i)=>-g.span/2+i*g.span/(n-1)-g.com);
    const xb=lx.reduce((a,b)=>a+b,0)/n,ss=lx.reduce((a,b)=>a+(b-xb)**2,0);
    // State [x,y,theta,vx,vy,omega, (z,vz,w)*n, motor work, dissipation, pitch integral].
    const W=6+3*n,D=W+1,P=W+2;
    const s=Array(P+1).fill(0),loads=lx.map(l=>M*G/n-M*G*xb*(l-xb)/ss);
    const anchor=loads.map((f,i)=>{if(f<=0||f/ks>.18)throw Error("invalid static suspension");s[6+3*i]=g.radius-(f+mw*G)/Kt;return s[6+3*i]+rest-f/ks+offset;});
    const ab=anchor.reduce((a,b)=>a+b,0)/n;s[2]=lx.reduce((a,l,i)=>a+(l-xb)*(anchor[i]-ab),0)/ss;s[1]=ab-s[2]*xb;
    function dynamics(q){
      const [x,y,th,vx,vy,om]=q,d=Array(q.length).fill(0);d[0]=vx;d[1]=vy;d[2]=om;
      let fx=-.65*vx*Math.abs(vx),fy=-M*G,moment=0,power=0,diss=.65*Math.abs(vx)**3,normalSum=0,slipSum=0,maxSlip=0,minClear=Infinity,maxComp=0,maxExt=0,maxPen=0,energy=.5*Mx*vx*vx+.5*M*vy*vy+.5*I*om*om+M*G*y;
      for(let i=0;i<n;i++){
        const j=6+3*i,z=q[j],vz=q[j+1],w=q[j+2],l=lx[i],xi=x+l,grade=slope(xi),norm=Math.hypot(1,grade);
        const compression=rest-(y+l*th-offset-z),cdot=vz-vy-l*om;
        // Bilateral spring; no hard bump stop in the ODE. Excess travel is a validity event.
        const Fs=ks*compression+cs*cdot;
        const pen=Math.max(0,h(xi)+g.radius-z),pdot=grade*vx-vz;
        const load=pen>0?Math.max(0,Kt*pen+Ct*pdot):0,N=load*norm;
        const vt=(vx+grade*vz)/norm,slip=g.radius*w-vt;
        const ft=mu*N*Math.tanh(slip/slipScale);
        // Torque acts between clockwise wheel and counterclockwise chassis.
        const relative=w+om, motor=drive?Math.min(28*g.gear/n,3400/(n*Math.max(1,Math.abs(relative)))):0;
        const bearing=.07*w;
        fx+=-load*grade+ft/norm;fy+=Fs;moment+=l*Fs+motor;
        d[j]=vz;d[j+1]=(-mw*G-Fs+load+ft*grade/norm)/mw;d[j+2]=(motor-g.radius*ft-bearing)/J;
        power+=motor*relative;
        diss+=cs*cdot*cdot+(load-Kt*pen)*pdot+ft*slip+bearing*w;
        energy+=.5*mw*vz*vz+.5*J*w*w+mw*G*z+.5*ks*compression*compression+.5*Kt*pen*pen;
        normalSum+=load;slipSum+=Math.abs(slip);maxSlip=Math.max(maxSlip,Math.abs(slip));maxComp=Math.max(maxComp,compression);maxExt=Math.max(maxExt,-compression);maxPen=Math.max(maxPen,pen);
      }
      for(const f of [-.48,-.24,0,.24,.48]){const l=f*g.span-g.com;minClear=Math.min(minClear,y+l*th-offset-h(x+l));}
      d[3]=fx/Mx;d[4]=fy/M;d[5]=moment/I;d[W]=power;d[D]=diss;d[P]=Math.abs(th);
      return {d,energy,power,diss,normalSum,slip:slipSum/n,maxSlip,minClear,maxComp,maxExt,maxPen};
    }
    return {initial:s,dynamics,mass:Mx,indices:{W,D,P},staticLoads:loads};
  }
  function simulate(g,seed,{kind="rough",drive=true,dt=1/480,duration=7,target=18,record=false,initialPerturbation=0}={}){
    if(!Number.isFinite(dt)||dt<=0||dt>1/120||!Number.isFinite(duration)||duration<=0||duration>30||!Number.isFinite(target)||target<=0||!Number.isFinite(initialPerturbation)||Math.abs(initialPerturbation)>.08)throw Error("invalid simulation options");
    const m=model(g,seed,{kind,drive}),{W,D,P}=m.indices;
    let q=m.initial.slice();q[1]+=initialPerturbation;
    const init=m.dynamics(q);let info=init,t=0,reason=null,steps=0,peakPower=0,peakSlip=0,peakTravel=0,minClear=Infinity,minDiss=Infinity,maxResidual=0,maxX=0,frames=[];
    const add=(s,d,h)=>s.map((v,i)=>v+h*d[i]);
    function rk(s,h,a){const b=m.dynamics(add(s,a.d,h/2)),c=m.dynamics(add(s,b.d,h/2)),e=m.dynamics(add(s,c.d,h));return s.map((v,i)=>v+h*(a.d[i]+2*b.d[i]+2*c.d[i]+e.d[i])/6);}
    function event(s,a){if(a.minClear<.025)return "chassis-strike";if(Math.abs(s[2])>.40)return "pitch-envelope";if(a.maxPen>.18)return "tire-envelope";if(a.maxComp>.23||a.maxExt>.20)return "travel-envelope";if(s[0]<-3)return "reverse-out";if(s[0]>=target)return "goal";return null;}
    function observe(s,a){peakPower=Math.max(peakPower,Math.abs(a.power));peakSlip=Math.max(peakSlip,a.maxSlip);peakTravel=Math.max(peakTravel,a.maxComp,a.maxExt);minClear=Math.min(minClear,a.minClear);minDiss=Math.min(minDiss,a.diss);maxX=Math.max(maxX,s[0]);maxResidual=Math.max(maxResidual,Math.abs(a.energy-init.energy-s[W]+s[D]));}
    observe(q,info);reason=event(q,info);
    while(t<duration-1e-12&&!reason){
      let step=Math.min(dt,duration-t),next=rk(q,step,info),ni=m.dynamics(next),ev=event(next,ni);
      if(ev){let lo=0,hi=step;for(let j=0;j<18&&hi-lo>1e-7;j++){const z=(lo+hi)/2,p=rk(q,z,info),b=m.dynamics(p);if(event(p,b))hi=z;else lo=z;}step=hi;next=rk(q,step,info);ni=m.dynamics(next);ev=event(next,ni);}
      if(!next.every(Number.isFinite))throw Error("nonfinite dynamics");q=next;info=ni;t+=step;steps++;reason=ev;observe(q,info);
      if(record&&steps%Math.max(1,Math.round(.05/dt))===0)frames.push({t,x:q[0],y:q[1],pitch:q[2],vx:q[3],wheelZ:Array.from({length:g.n},(_,i)=>q[6+3*i]),wheelOmega:Array.from({length:g.n},(_,i)=>q[8+3*i])});
      if(steps>1e6)throw Error("step budget");
    }
    const reached=reason==="goal",failed=!!reason&&!reached,meanPitch=q[P]/Math.max(t,.001),score=Math.min(maxX,target)+(reached?5+.6*(duration-t):0)-Math.max(0,q[W])/9000-1.2*meanPitch-(failed?9:0);
    return {score,reason:reason??"timeout",reached,failed,distance:maxX,time:t,work:q[W],dissipation:q[D],initialEnergy:init.energy,finalEnergy:info.energy,maxEnergyResidual:maxResidual,relativeEnergyResidual:maxResidual/Math.max(1,Math.abs(q[W]),Math.abs(q[D]),Math.abs(init.energy)),peakPower,peakSlip,peakTravel,minClear,minDiss,meanPitch,steps,mass:m.mass,final:q,frames};
  }
  function selfTest(){
    const checks={},test=(name,ok)=>checks[name]=!!ok;
    const m=model(base,1,{kind:"flat",drive:false}),a=m.dynamics(m.initial);
    test("static force balance",Math.max(...a.d.slice(0,m.indices.W).map(Math.abs))<1e-8);
    test("static weight",Math.abs(a.normalSum-m.mass*G)<1e-8);
    const offsetModel=model({...base,com:.12},1,{kind:"flat",drive:false}),offsetState=offsetModel.dynamics(offsetModel.initial);
    test("off-center static balance",Math.max(...offsetState.d.slice(0,offsetModel.indices.W).map(Math.abs))<1e-8);
    const still=simulate(base,1,{kind:"flat",drive:false,duration:2}),bounce=simulate(base,1,{kind:"flat",drive:false,duration:3,initialPerturbation:.04}),move=simulate(base,1,{kind:"flat",duration:5}),ice=simulate(base,1,{kind:"low-grip",duration:5});
    test("unpowered equilibrium",still.distance<1e-8&&!still.failed&&Math.abs(still.work)<1e-9);
    test("passive damping",bounce.dissipation>0&&bounce.finalEnergy<bounce.initialEnergy&&bounce.minDiss>=-1e-7);
    test("powered translation and wheel spin",move.distance>5&&move.work>0&&move.final[8]>0);
    test("low grip increases slip",ice.peakSlip>move.peakSlip);
    test("motor power bound",[move,ice].every(x=>x.peakPower<=3400+1e-6));
    test("energy accounting",[still,bounce,move,ice].every(x=>x.relativeEnergyResidual<.002));
    const rough=simulate(base,7,{kind:"rough",duration:5}),fine=simulate(base,7,{kind:"rough",duration:5,dt:1/960});
    test("rough terrain convergence",rough.reason===fine.reason&&Math.abs(rough.score-fine.score)<.05);
    test("determinism",JSON.stringify(rough)===JSON.stringify(simulate(base,7,{kind:"rough",duration:5})));
    let rejected=false;try{model({...base,n:4},1)}catch(e){rejected=true;}test("overlapping wheels rejected",rejected);
    const brief=x=>Object.fromEntries(["score","reason","distance","work","dissipation","peakSlip","peakPower","relativeEnergyResidual","minDiss"].map(k=>[k,x[k]]));
    return {checks,passed:Object.values(checks).filter(Boolean).length,total:Object.keys(checks).length,examples:{still:brief(still),bounce:brief(bounce),flat:brief(move),lowGrip:brief(ice),rough:brief(rough),fine:brief(fine)}};
  }
  function benchmark(){
    const configs={base,soft:{...base,stiffness:22000,damping:1600},hard:{...base,stiffness:65000,damping:3600},three:{...base,n:3,span:1.9},four:{...base,n:4,span:2.65}};
    const kinds=["flat","rough","ridge","washboard","low-grip"],seeds=[31,47,83],rows=[];
    const mini=x=>Object.fromEntries(["score","reason","distance","time","work","dissipation","peakPower","peakSlip","peakTravel","relativeEnergyResidual","minDiss","minClear"].map(k=>[k,x[k]]));
    for(const [name,g] of Object.entries(configs))for(const kind of kinds)for(const seed of seeds){
      const coarse=simulate(g,seed,{kind,dt:1/240}),mid=simulate(g,seed,{kind,dt:1/480}),fine=simulate(g,seed,{kind,dt:1/960});
      rows.push({name,kind,seed,coarse:mini(coarse),mid:mini(mid),fine:mini(fine)});
    }
    const compare=(a,b)=>{const d=rows.map(r=>Math.abs(r[a].score-r[b].score));return {mean:d.reduce((s,x)=>s+x,0)/d.length,max:Math.max(...d),abovePoint05:d.filter(x=>x>.05).length,eventDisagreements:rows.filter(r=>r[a].reason!==r[b].reason).length};};
    const summary=Object.fromEntries(Object.keys(configs).map(name=>{const r=rows.filter(x=>x.name===name),mean=k=>r.reduce((s,x)=>s+x.fine[k],0)/r.length;return [name,{mass:model(configs[name],31).mass,score:mean("score"),distance:mean("distance"),work:mean("work"),peakSlip:mean("peakSlip"),goals:r.filter(x=>x.fine.reason==="goal").length,invalid:r.filter(x=>!["goal","timeout"].includes(x.fine.reason)).length}];}));
    return {configs,kinds,seeds,cases:rows.length,simulationRuns:rows.length*3,convergence:{coarseVsFine:compare("coarse","fine"),midVsFine:compare("mid","fine")},maxRelativeEnergyResidual:Math.max(...rows.flatMap(r=>["coarse","mid","fine"].map(k=>r[k].relativeEnergyResidual))),maxPeakPower:Math.max(...rows.flatMap(r=>["coarse","mid","fine"].map(k=>r[k].peakPower))),summary,rows};
  }
  return {base,model,simulate,selfTest,benchmark};
}
