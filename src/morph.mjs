// V0.2 reduced-order 2D multi-axle morphology experiment. Not a full rigid-body or wheel-slip solver.
export function createMorphLab(){
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  function rng(seed){let s=seed>>>0;return ()=>{s=(s+0x6d2b79f5)>>>0;let t=Math.imul(s^(s>>>15),1|s);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
  const ranges={span:[1.4,3.1],radius:[.23,.52],gear:[2.5,7],stiffness:[22000,70000],damping:[1200,4500],com:[-.13,.13]};
  const keys=Object.keys(ranges);
  function repair(g){const z={};z.n=clamp(Math.round(g.n),2,4);for(const k of keys)z[k]=clamp(g[k],...ranges[k]);z.span=Math.max(z.span,2*(z.n-1)*(ranges.radius[0]+.025));z.radius=Math.min(z.radius,z.span/(2*(z.n-1))-.025);return z;}
  const fixed=repair({n:2,span:1.7,radius:.38,gear:4.5,stiffness:50000,damping:2800,com:0});
  function sample(r,variable=true){return repair({n:variable?2+Math.floor(r()*3):2,...Object.fromEntries(keys.map(k=>[k,ranges[k][0]+r()*(ranges[k][1]-ranges[k][0])]))});}
  function mutate(g,r,variable=true){const z={...g};if(variable&&r()<.25)z.n+=r()<.5?-1:1;for(const k of keys){if(r()<.6)z[k]+=(r()+r()+r()-1.5)*.32*(ranges[k][1]-ranges[k][0]);}return repair(z);}
  function geometry(g){const n=g.n, mass=86+13*n+12*g.span+25*n*g.radius*g.radius;const axles=Array.from({length:n},(_,i)=>-g.span/2+i*g.span/(n-1)-g.com);
    const inertia=mass*(g.span*g.span+.5*.5)/12+mass*g.com*g.com;
    return {mass,inertia,axles};
  }
  function ground(seed,kind){
    const r=rng(seed), family=kind??["rough","ridge","washboard"][seed%3], phase=r()*Math.PI*2, amp=.8+.4*r(), mu=.48+.32*r();
    const smooth=z=>{z=clamp(z,0,1);return z*z*(3-2*z);};
    const h=x=>{const z=x-3,w=smooth(z/2);if(z<=0||family==="flat")return 0;
      if(family==="ridge")return w*amp*(.95*Math.exp(-((z-5)**2)/3.2)+.68*Math.exp(-((z-12)**2)/2.2)+.11*Math.sin(2*z+phase));
      if(family==="washboard")return w*amp*(.22*Math.sin(3.2*z+phase)+.17*Math.sin(5.3*z));
      return w*amp*(.43*Math.sin(.86*z+phase)+.22*Math.sin(2.1*z+phase/2));
    };
    return {h,mu:family==="flat"?.75:mu,family};
  }
  function simulate(g,seed,{dt=1/120,duration=10,kind,drive=true,record=false}={}){
    if(!Number.isFinite(dt)||dt<1/1000||dt>1/60||!Number.isFinite(duration)||duration<=0||duration>60)throw Error("invalid simulation settings");
    if(!g||!["n",...keys].every(k=>Number.isFinite(g[k]))||["n",...keys].some(k=>Math.abs(repair(g)[k]-g[k])>1e-10))throw Error("invalid geometry");
    const {mass:m,inertia:I,axles}=geometry(g), {h,mu,family}=ground(seed,kind),gravity=9.81,power=3400,torque=28,target=22;
    const k=g.stiffness/g.n,c=g.damping/g.n;
    let x=0,y=g.radius+.23-m*gravity/g.stiffness,th=0,vx=0,vy=0,omega=0,t=0,work=0,tilt=0,maxX=0,failed=false,reached=false,reason=null,frames=[],peakPower=0,maxPen=0;
    for(let i=0;i<Math.ceil(duration/dt);i++){
      const step=Math.min(dt,duration-t);if(step<=0)break;
      const co=Math.cos(th),si=Math.sin(th);
      let fx=-.65*vx*Math.abs(vx),fy=-m*gravity,moment=0,instantPower=0;
      for(const lx of axles){
        const rx=co*lx+si*.23,ry=si*lx-co*.23,cx=x+rx,cy=y+ry;
        const slope=(h(cx+.001)-h(cx-.001))/.002,inv=1/Math.hypot(1,slope),nx=-slope*inv,ny=inv,tx=inv,ty=slope*inv;
        const pen=g.radius-(cy-h(cx))*inv;if(pen<=0)continue;
        maxPen=Math.max(maxPen,pen);
        const px=rx-g.radius*nx,py=ry-g.radius*ny;
        const ux=vx-omega*py,uy=vy+omega*px,vn=ux*nx+uy*ny,vt=ux*tx+uy*ty;
        const normal=Math.max(0,k*pen-c*vn);
        // Fixed total motor budget divided among installed axles, not multiplied by axle count.
        // Smooth low-speed traction; power uses the same point velocity as the applied force.
        const effort=drive?Math.min(torque*g.gear/g.radius,power/Math.max(.65,Math.abs(vt)))/g.n:0;
        const tr=Math.min(effort,mu*normal),roll=.024*normal*Math.tanh(vt/.25);
        const tang=clamp(tr-roll,-mu*normal,mu*normal),ax=normal*nx+tang*tx,ay=normal*ny+tang*ty;
        fx+=ax;fy+=ay;moment+=px*ay-py*ax;instantPower+=Math.max(0,tr*vt);
      }
      peakPower=Math.max(peakPower,instantPower);work+=instantPower*step;
      vx+=fx/m*step;vy+=fy/m*step;omega+=moment/I*step;x+=vx*step;y+=vy*step;th+=omega*step;t+=step;
      if(![x,y,th,vx,vy,omega,work].every(Number.isFinite))throw Error("nonfinite dynamics");
      maxX=Math.max(maxX,x);tilt+=Math.abs(th)*step;
      if(record&&i%8===0)frames.push({t,x,y,theta:th,vx,vy,work});
      // Conservative chassis clearance checks at five points.
      for(const q of [-.48,-.24,0,.24,.48]){
        const lx=q*g.span-g.com, bx=x+Math.cos(th)*lx+Math.sin(th)*.15,by=y+Math.sin(th)*lx-Math.cos(th)*.15;
        if(by<h(bx)+.025){failed=true;reason="chassis-strike";}
      }
      if(Math.abs(th)>1.25){failed=true;reason="overturn";}
      if(maxPen>.65*g.radius){failed=true;reason="excess-penetration";}
      if(x< -3){failed=true;reason="reverse-out";}
      if(failed)break;
      if(x>=target){reached=true;break;}
    }
    const meanPitch=tilt/Math.max(t,.001),score=Math.min(maxX,target)+(reached?5+(duration-t)*.7:0)-work/9000-1.2*meanPitch-(failed?9:0);
    return {score,distance:maxX,time:t,work,meanPitch,failed,reached,reason,family,mass:m,peakPower,maxPen,frames};
  }
  function evaluate(g,seeds,options){if(!seeds.length)throw Error("no terrain seeds");const runs=seeds.map(s=>simulate(g,s,options)),mean=k=>runs.reduce((a,b)=>a+b[k],0)/runs.length;
    return {score:mean("score"),pitch:mean("meanPitch"),work:mean("work"),reached:runs.filter(x=>x.reached).length/runs.length,failed:runs.filter(x=>x.failed).length/runs.length};
  }
  // Descriptor: axle count and average absolute chassis pitch (radians).
  const cuts=[.05,.10,.18,.30];
  function cell(g,v){return `${g.n}:${cuts.filter(x=>v.pitch>=x).length}`;}
  function search({seed=1,budget=120,train=[12,13,14,24,25,26],method="map"}={}){
    if(!["map","evolution","fixed","random"].includes(method)||!Number.isInteger(budget)||budget<12||!train.length)throw Error("invalid search");
    const r=rng(seed), archive=new Map(),elite=[];let best=null,history=[];
    for(let i=0;i<budget;i++){
      const variable=method!=="fixed";let g;
      if(i===0)g={...fixed};
      else if(i<12||method==="random"||r()<.18)g=sample(r,variable);
      else {const parents=method==="map"?Array.from(archive.values()):elite;g=mutate(parents[Math.floor(r()*parents.length)].g,r,variable);}
      const value=evaluate(g,train),entry={g,value},id=cell(g,value);
      if(!archive.has(id)||archive.get(id).value.score<value.score)archive.set(id,entry);
      elite.push(entry);elite.sort((a,b)=>b.value.score-a.value.score);if(elite.length>8)elite.pop();
      if(!best||value.score>best.value.score)best=entry;
      if((i+1)%12===0||i===budget-1)history.push({n:i+1,best:best.value.score,cells:archive.size});
    }
    return {method,best,archive:Object.fromEntries(archive),history,evaluations:budget,simulationRuns:budget*train.length};
  }
  function benchmark({seeds=[201,202,203,204,205,206,207,208],budget=120,train=[12,13,14,24,25,26],test=[60,61,62,72,73,74]}={}){
    if(train.some(s=>test.includes(s)))throw Error("overlapping train/test");
    const methods=["random","fixed","evolution","map"],rows=[];
    for(const seed of seeds){
      const row={seed};
      for(const method of methods){
        const s=search({seed,budget,train,method}),v=evaluate(s.best.g,test);
        const heldFine=evaluate(s.best.g,test,{dt:1/240});
        row[method]={train:s.best.value.score,test:v.score,testFine:heldFine.score,reached:v.reached,failed:v.failed,cells:Object.keys(s.archive).length,n:s.best.g.n,g:s.best.g,archive:s.archive,history:s.history};
      }
      rows.push(row);
    }
    const mean=(method,k)=>rows.reduce((a,row)=>a+row[method][k],0)/rows.length;
    const summary=Object.fromEntries(methods.map(m=>[m,Object.fromEntries(["train","test","testFine","reached","failed","cells"].map(k=>[k,mean(m,k)]))]));
    const compare=(a,b)=>{const d=rows.map(r=>r[a].test-r[b].test);return {wins:d.filter(x=>x>1e-9).length,ties:d.filter(x=>Math.abs(x)<=1e-9).length,losses:d.filter(x=>x< -1e-9).length,mean:d.reduce((x,y)=>x+y,0)/d.length,min:Math.min(...d),max:Math.max(...d)};};
    return {seeds,train,test,budget,searchRuns:seeds.length*methods.length*budget*train.length,summary,compare:{mapVsRandom:compare("map","random"),mapVsEvolution:compare("map","evolution"),evolutionVsFixed:compare("evolution","fixed")},rows};
  }
  function selfTest(){
    const checks={};const assert=(n,b)=>{checks[n]=!!b;};
    const r=rng(99);let g=fixed,counts=new Set();
    for(let i=0;i<2000;i++){g=mutate(g,r);counts.add(g.n);if(g.span/(g.n-1)<2*g.radius+.05-1e-10)throw Error("overlap");}
    assert("all wheel counts reachable",counts.size===3);
    assert("geometry gap after mutation",true);
    const a=simulate(fixed,12),b=simulate(fixed,12);assert("determinism",JSON.stringify(a)===JSON.stringify(b));
    const still=simulate(fixed,1,{kind:"flat",drive:false,duration:3});assert("flat equilibrium",still.distance<1e-6&&!still.failed&&still.work===0);
    const powered=simulate(fixed,1,{kind:"flat",duration:3});assert("flat powered motion",powered.distance>2&&!powered.failed&&powered.work>0);
    const mass2=geometry(repair({...fixed,n:2,span:2.8})).mass,mass4=geometry(repair({...fixed,n:4,span:2.8})).mass;
    assert("extra wheels increase mass",mass4>mass2);
    assert("total motor power bound",powered.peakPower<=3400+1e-6&&a.peakPower<=3400+1e-6);
    const samples=["random","fixed","evolution","map"].map(method=>search({seed:1,method,budget:24,train:[12,13,14]}));
    assert("matched evaluations",samples.every(x=>x.evaluations===24&&x.simulationRuns===72));
    assert("fixed topology",Object.values(samples[1].archive).every(x=>x.g.n===2));
    assert("archive keyed correctly",samples.every(s=>Object.entries(s.archive).every(([id,x])=>id===cell(x.g,x.value))));
    assert("monotonic best",samples.every(s=>s.history.every((v,i)=>i===0||v.best>=s.history[i-1].best)));
    const fine=simulate(fixed,12,{dt:1/240});
    return {checks,passed:Object.values(checks).filter(Boolean).length,total:Object.keys(checks).length,timestep:{coarse:a.score,fine:fine.score,absoluteDelta:Math.abs(a.score-fine.score)},flat:{distance:powered.distance,work:powered.work,peakPower:powered.peakPower},mass:{two:mass2,four:mass4}};
  }
  return {ranges,fixed,repair,geometry,ground,simulate,evaluate,search,benchmark,selfTest};
}
