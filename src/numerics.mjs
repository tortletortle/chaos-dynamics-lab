// V0.3: controlled numerical audit of V0.2; no new physical degrees of freedom.
export function createNumericsLab(){
  // Preserve V0.2 force law and geometry; change only numerical integration and event timing.
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  function rng(seed){let s=seed>>>0;return ()=>{s=(s+0x6d2b79f5)>>>0;let t=Math.imul(s^(s>>>15),1|s);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
  const fixed={n:2,span:1.7,radius:.38,gear:4.5,stiffness:50000,damping:2800,com:0};
  function ground(seed,kind){
    const r=rng(seed),family=kind??["rough","ridge","washboard"][seed%3],phase=r()*Math.PI*2,amp=.8+.4*r(),mu=.48+.32*r();
    const smooth=z=>{z=clamp(z,0,1);return z*z*(3-2*z);};
    return {family,mu:family==="flat"?.75:mu,h:x=>{const z=x-3,w=smooth(z/2);if(z<=0||family==="flat")return 0;if(family==="ridge")return w*amp*(.95*Math.exp(-((z-5)**2)/3.2)+.68*Math.exp(-((z-12)**2)/2.2)+.11*Math.sin(2*z+phase));if(family==="washboard")return w*amp*(.22*Math.sin(3.2*z+phase)+.17*Math.sin(5.3*z));return w*amp*(.43*Math.sin(.86*z+phase)+.22*Math.sin(2.1*z+phase/2));}};
  }
  function model(g,seed,{kind,drive=true}={}){
    if(!g||!Number.isInteger(g.n)||g.n<2||g.n>4||!["span","radius","gear","stiffness","damping","com"].every(k=>Number.isFinite(g[k])))throw Error("invalid genome");
    if(g.radius<=0||g.span<=0||g.stiffness<=0||g.damping<0||g.gear<=0||g.span/(g.n-1)<2*g.radius+.05-1e-9)throw Error("invalid geometry");
    const {h,mu,family}=ground(seed,kind),mass=86+13*g.n+12*g.span+25*g.n*g.radius*g.radius,I=mass*(g.span*g.span+.25)/12+mass*g.com*g.com;
    const axles=Array.from({length:g.n},(_,i)=>-g.span/2+i*g.span/(g.n-1)-g.com),k=g.stiffness/g.n,c=g.damping/g.n;
    // state: x, y, pitch, vx, vy, angular velocity, motor work, integral absolute pitch.
    const initial=[0,g.radius+.23-mass*9.81/g.stiffness,0,0,0,0,0,0];
    function forces(s){
      const [x,y,th,vx,vy,omega]=s,co=Math.cos(th),si=Math.sin(th);
      let fx=-.65*vx*Math.abs(vx),fy=-mass*9.81,moment=0,power=0,maxPen=0;
      for(const lx of axles){
        const rx=co*lx+si*.23,ry=si*lx-co*.23,cx=x+rx,cy=y+ry;
        const slope=(h(cx+.001)-h(cx-.001))/.002,inv=1/Math.hypot(1,slope),nx=-slope*inv,ny=inv,tx=inv,ty=slope*inv;
        const pen=g.radius-(cy-h(cx))*inv;maxPen=Math.max(maxPen,pen);if(pen<=0)continue;
        const px=rx-g.radius*nx,py=ry-g.radius*ny,ux=vx-omega*py,uy=vy+omega*px,vn=ux*nx+uy*ny,vt=ux*tx+uy*ty;
        const normal=Math.max(0,k*pen-c*vn),effort=drive?Math.min(28*g.gear/g.radius,3400/Math.max(.65,Math.abs(vt)))/g.n:0;
        const tr=Math.min(effort,mu*normal),roll=.024*normal*Math.tanh(vt/.25),tang=clamp(tr-roll,-mu*normal,mu*normal),ax=normal*nx+tang*tx,ay=normal*ny+tang*ty;
        fx+=ax;fy+=ay;moment+=px*ay-py*ax;power+=Math.max(0,tr*vt);
      }
      return {d:[vx,vy,omega,fx/mass,fy/mass,moment/I,power,Math.abs(th)],maxPen,power};
    }
    function event(s,pen){
      const [x,y,th]=s,co=Math.cos(th),si=Math.sin(th);
      let clearance=Infinity;
      for(const q of [-.48,-.24,0,.24,.48]){const lx=q*g.span-g.com,bx=x+co*lx+si*.15,by=y+si*lx-co*.15;clearance=Math.min(clearance,by-h(bx)-.025);}
      if(pen===undefined)pen=forces(s).maxPen;
      // Identical failure precedence to V0.2; continuous event surfaces.
      if(x< -3)return "reverse-out";
      if(pen>.65*g.radius)return "excess-penetration";
      if(Math.abs(th)>1.25)return "overturn";
      if(clearance<0)return "chassis-strike";
      if(x>=22)return "goal";
      return null;
    }
    return {initial,forces,event,family,mass};
  }
  const add=(a,d,h)=>a.map((v,i)=>v+h*d[i]);
  function simulate(g,seed,{mode="adaptive",dt=1/120,duration=10,kind,drive=true,tol=1,record=false}={}){
    if(!["legacy","adaptive","midpoint"].includes(mode)||!Number.isFinite(dt)||dt<=0||dt>1/30||!Number.isFinite(duration)||duration<=0||duration>60||!Number.isFinite(tol)||tol<=0)throw Error("invalid settings");
    const m=model(g,seed,{kind,drive}),minStep=Math.min(dt,1/7680);
    let s=m.initial.slice(),t=0,maxX=0,maxPen=0,peakPower=0,accepted=0,rejected=0,limited=0,forceCalls=0,reason=null,frames=[];
    const f=y=>{forceCalls++;return m.forces(y);};
    function half(y,h){const a=f(y),mid=add(y,a.d,h/2),b=f(mid);return add(y,b.d,h);}
    const pair=(y,h)=>{const mid=half(y,h/2);return {mid,end:half(mid,h/2)};};
    // Compare one midpoint step to two half steps (second-order embedded estimate).
    const abs=[2e-5,2e-5,2e-5,2e-4,2e-4,2e-4,.025,2e-5],rel=2e-5;
    const error=(a,b)=>Math.max(...a.map((v,i)=>Math.abs(v-b[i])/(3*tol*(abs[i]+rel*Math.max(Math.abs(v),Math.abs(b[i]))))));
    function observe(y){const a=f(y);maxPen=Math.max(maxPen,a.maxPen);peakPower=Math.max(peakPower,a.power);maxX=Math.max(maxX,y[0]);return m.event(y,a.maxPen);}
    let proposed=dt;
    while(t<duration-1e-12&&!reason){
      let h=Math.min(mode==="adaptive"?proposed:dt,duration-t),next,mid,event;
      if(mode==="legacy"){
        const a=f(s),d=a.d;maxPen=Math.max(maxPen,a.maxPen);peakPower=Math.max(peakPower,a.power);
        next=s.slice();for(let i=3;i<=5;i++)next[i]+=h*d[i];
        for(let i=0;i<3;i++)next[i]+=h*next[i+3];
        next[6]+=h*d[6];next[7]+=h*Math.abs(next[2]);
        maxX=Math.max(maxX,next[0]);event=m.event(next,maxPen);
      }else{
        if(mode==="adaptive"){
          const p=pair(s,h);const e=error(half(s,h),p.end);
          if(e>1&&h>minStep*(1+1e-9)){proposed=Math.max(minStep,h*clamp(.85*Math.pow(e,-1/3),.2,.8));rejected++;continue;}
          if(e>1)limited++;
          proposed=Math.min(dt,Math.max(minStep,h*clamp(e===0?2:.9*Math.pow(e,-1/3),.5,2)));
          next=p.end;mid=p.mid;
        }else{next=half(s,h);mid=half(s,h/2);}
        // Inspect midpoint and endpoint; localize the first sampled terminal crossing.
        const midEvent=m.event(mid),endEvent=m.event(next);
        if(midEvent||endEvent){
          let lo=0,hi=midEvent?h/2:h;
          const propagate=u=>mode==="adaptive"?pair(s,u).end:half(s,u);
          for(let j=0;j<18&&hi-lo>1e-7;j++){const z=(lo+hi)/2;if(m.event(propagate(z)))hi=z;else lo=z;}
          next=propagate(hi);h=hi;event=m.event(next);
          observe(propagate(h/2));observe(next);
        }else{observe(mid);observe(next);}
      }
      if(!next.every(Number.isFinite))throw Error("nonfinite state");
      t+=h;s=next;accepted++;reason=event??null;
      if(record&&accepted%20===0)frames.push({t,state:s.slice()});
      if(accepted+rejected>2e6)throw Error("step budget exceeded");
    }
    const reached=reason==="goal",failed=!!reason&&!reached,meanPitch=s[7]/Math.max(t,.001),score=Math.min(maxX,22)+(reached?5+(duration-t)*.7:0)-s[6]/9000-1.2*meanPitch-(failed?9:0);
    return {score,reason:reason??"timeout",reached,failed,distance:maxX,time:t,work:s[6],meanPitch,peakPower,maxPen,accepted,rejected,limited,forceCalls,final:s,frames};
  }
  function selfTest(){
    const a=simulate(fixed,12),b=simulate(fixed,12),still=simulate(fixed,1,{kind:"flat",drive:false,duration:3}),moving=simulate(fixed,1,{kind:"flat",duration:3}),goal=simulate(fixed,1,{kind:"flat",duration:10});
    const r=simulate(fixed,12,{dt:1/960,mode:"midpoint"}),checks={
      deterministic:JSON.stringify(a)===JSON.stringify(b),
      equilibrium:still.distance<1e-7&&still.work===0&&!still.failed,
      poweredMotion:moving.distance>2&&moving.work>0&&!moving.failed,
      powerLimit:[a,moving,goal].every(x=>x.peakPower<=3400+1e-6),
      localizedGoal:goal.reached&&Math.abs(goal.final[0]-22)<1e-5,
      knownCaseConvergence:Math.abs(a.score-r.score)<.05&&a.reason===r.reason,
      noToleranceFloor:[a,moving,goal].every(x=>x.limited===0),
      finiteResults:[a,still,moving,goal,r].every(x=>Number.isFinite(x.score))
    };
    return {checks,passed:Object.values(checks).filter(Boolean).length,total:Object.keys(checks).length,knownCase:{adaptive:a.score,reference:r.score,delta:Math.abs(a.score-r.score),reason:a.reason},goalError:Math.abs(goal.final[0]-22),adaptiveSteps:{accepted:a.accepted,rejected:a.rejected,limited:a.limited,forceCalls:a.forceCalls}};
  }
  function audit(previous){
    const methods=["random","fixed","evolution","map"],cases=[];
    for(const row of previous.rows)for(const method of methods)for(const seed of previous.test){
      const g=row[method].g;
      const l120=simulate(g,seed,{mode:"legacy",dt:1/120}),l240=simulate(g,seed,{mode:"legacy",dt:1/240}),
      a120=simulate(g,seed,{dt:1/120}),a240=simulate(g,seed,{dt:1/240}),ref=simulate(g,seed,{mode:"midpoint",dt:1/960}),ref2=simulate(g,seed,{mode:"midpoint",dt:1/1920});
      const mini=x=>Object.fromEntries(["score","reason","time","distance","work","peakPower","maxPen","accepted","rejected","limited","forceCalls"].map(k=>[k,x[k]]));
      cases.push({searchSeed:row.seed,method,terrainSeed:seed,l120:mini(l120),l240:mini(l240),a120:mini(a120),a240:mini(a240),ref:mini(ref),ref2:mini(ref2)});
    }
    const stat=(a,b)=>{const d=cases.map(x=>Math.abs(x[a].score-x[b].score)),sorted=d.slice().sort((x,y)=>x-y);return {mean:d.reduce((x,y)=>x+y,0)/d.length,p95:sorted[Math.ceil(.95*d.length)-1],max:Math.max(...d),aboveHalf:d.filter(x=>x>.5).length,eventDisagreements:cases.filter(x=>x[a].reason!==x[b].reason).length};};
    let replayMax=0;
    const joined=[];
    for(const row of previous.rows)for(const method of methods){
      const c=cases.filter(x=>x.searchSeed===row.seed&&x.method===method),mean=k=>c.reduce((a,b)=>a+b[k].score,0)/c.length;
      replayMax=Math.max(replayMax,Math.abs(mean("l120")-row[method].test),Math.abs(mean("l240")-row[method].testFine));
      joined.push({searchSeed:row.seed,method,legacy:mean("l120"),legacyFine:mean("l240"),adaptive:mean("a120"),reference:mean("ref2")});
    }
    const methodMeans=Object.fromEntries(methods.map(m=>{const c=joined.filter(x=>x.method===m);return [m,{adaptive:c.reduce((a,b)=>a+b.adaptive,0)/c.length,reference:c.reduce((a,b)=>a+b.reference,0)/c.length}];}));
    return {cases:cases.length,simulationRuns:cases.length*6,replayMax,comparisons:{legacy120vs240:stat("l120","l240"),adaptive120vs240:stat("a120","a240"),adaptive120vsReference:stat("a120","ref2"),reference960vs1920:stat("ref","ref2")},toleranceFloorCases:cases.filter(x=>x.a120.limited||x.a240.limited).length,meanForceCalls: Object.fromEntries(["l120","a120","ref2"].map(k=>[k,cases.reduce((a,b)=>a+b[k].forceCalls,0)/cases.length])),methodMeans,joined,details:cases};
  }
  return {fixed,simulate,selfTest,audit};
}
