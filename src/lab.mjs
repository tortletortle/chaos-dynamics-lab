// Reduced-order planar chassis with two compliant wheel contacts; no wheel inertia or deformable soil.
export function createLab(){
  "use strict";
  const bounds={radius:[.25,.55],wheelbase:[1.1,1.9],gear:[2,7],stiffness:[15000,40000],damping:[700,2500],com:[-.19,.19]};
  const keys=Object.keys(bounds), clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  function rng(seed){let state=seed>>>0;return ()=>{state=(state+0x6D2B79F5)>>>0;let t=Math.imul(state^(state>>>15),1|state);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
  function sample(r){return Object.fromEntries(keys.map(k=>[k,bounds[k][0]+r()*(bounds[k][1]-bounds[k][0])]));}
  function mutate(g,r){return Object.fromEntries(keys.map(k=>{const [a,b]=bounds[k];return [k,clamp(g[k]+(r()<.65?(r()+r()+r()-1.5)*.24*(b-a):0),a,b)];}));}
  function terrain(seed){
    const r=rng(seed), a=.12+.40*r(), f=.65+.55*r(), p=r()*6.283, q=r()*6.283, mu=.45+.4*r();
    function h(x){const z=Math.max(0,x-2), w=Math.min(1,z/2); return w*w*(3-2*w)*a*(Math.sin(f*z+p)+.45*Math.sin(2.2*f*z+q));}
    return {h,mu};
  }
  function simulate(g,seed,{dt=1/120,duration=12,flat=false,drive=true,record=false}={}){
    for(const k of keys){const [a,b]=bounds[k];if(!Number.isFinite(g[k])||g[k]<a||g[k]>b)throw Error("invalid "+k);}
    if(!Number.isFinite(dt)||dt<1/1000||dt>1/60||!Number.isFinite(duration)||duration<=0||duration>60)throw Error("invalid time");
    const {h,mu}=flat?{h:()=>0,mu:.7}:terrain(seed);
    const m=120,G=9.81,I=m*(g.wheelbase**2+.6**2)/12, power=3200, torque=22, target=25;
    let x=0,y=g.radius+.25-m*G/(2*g.stiffness),th=0,vx=0,vy=0,om=0,energy=0,tilt=0,dist=0,time=0,failed=false,reached=false,frames=[];
    for(let i=0;i<Math.ceil(duration/dt);i++){
      const step=Math.min(dt,duration-time); if(step<=0)break;
      let fx=-.7*vx*Math.abs(vx),fy=-m*G,moment=0;
      const co=Math.cos(th),si=Math.sin(th);
      for(const side of [-1,1]){
        const lx=side*g.wheelbase/2-g.com,ly=-.25;
        const rx=co*lx-si*ly,ry=si*lx+co*ly,cx=x+rx,cy=y+ry;
        const slope=(h(cx+.001)-h(cx-.001))/.002, inv=1/Math.hypot(1,slope);
        const nx=-slope*inv,ny=inv,tx=inv,ty=slope*inv;
        const compression=g.radius-(cy-h(cx))*inv;
        if(compression<=0)continue;
        const wx=vx-om*ry,wy=vy+om*rx,vn=wx*nx+wy*ny,vt=wx*tx+wy*ty;
        const normal=Math.max(0,g.stiffness*compression-g.damping*vn);
        const effort=drive?Math.min(torque*g.gear/g.radius,power/Math.max(.5,Math.abs(vt)))/2:0;
        const traction=Math.min(effort,mu*normal);
        const rolling=.018*normal*Math.tanh(vt/.2);
        const tang=clamp(traction-rolling,-mu*normal,mu*normal);
        const ax=normal*nx+tang*tx,ay=normal*ny+tang*ty;
        fx+=ax;fy+=ay;moment+=(rx-g.radius*nx)*ay-(ry-g.radius*ny)*ax;
        energy+=Math.max(0,traction*vt)*step;
      }
      vx+=fx/m*step;vy+=fy/m*step;om+=moment/I*step;
      x+=vx*step;y+=vy*step;th+=om*step;time+=step;
      dist=Math.max(dist,x);tilt+=Math.abs(th)*step;
      if(record&&(i%8===0))frames.push({time,x,y,theta:th,vx,vy,energy});
      if(![x,y,th,vx,vy,om,energy].every(Number.isFinite))throw Error("nonfinite dynamics");
      // Reject chassis strikes and overturned bodies rather than rewarding numerical exploits.
      for(const side of [-1,1]){
        const lx=side*g.wheelbase*.43-g.com,ly=-.17;
        const bx=x+Math.cos(th)*lx-Math.sin(th)*ly,by=y+Math.sin(th)*lx+Math.cos(th)*ly;
        if(by<h(bx)+.025)failed=true;
      }
      if(Math.abs(th)>1.25||x< -3)failed=true;
      if(failed)break;
      if(x>=target){reached=true;break;}
    }
    const score=Math.min(dist,target)+(reached?4+Math.max(0,duration-time)*.6:0)-energy/12000-.3*tilt/Math.max(time,.001)-(failed?8:0);
    return {score,distance:dist,time,energy,failed,reached,meanAbsPitch:tilt/Math.max(time,.001),frames};
  }
  const fixed={radius:.4,wheelbase:1.5,gear:4.5,stiffness:27500,damping:1600,com:0};
  function evaluate(g,seeds,options){if(!seeds.length)throw Error("empty seeds");let s=0;for(const seed of seeds)s+=simulate(g,seed,options).score;return s/seeds.length;}
  function search({seed=1,population=20,generations=10,train=[11,17,23],method="evolution"}={}){
    if(!Number.isInteger(population)||population<4||!Number.isInteger(generations)||generations<1||!["evolution","random"].includes(method))throw Error("invalid search");
    const r=rng(seed);let pool=[],best=null,history=[],count=0;
    for(let gen=0;gen<generations;gen++){
      const elite=pool.slice(0,Math.max(2,Math.floor(population/4)));
      const next=Array.from({length:population},(_,i)=>gen===0?(i===0?{...fixed}:sample(r)):method==="random"||i<Math.ceil(population*.2)?sample(r):mutate(elite[Math.floor(r()*elite.length)].g,r));
      const scored=next.map(g=>({g,score:evaluate(g,train)}));count+=scored.length;
      pool=(method==="evolution"?pool.concat(scored):scored).sort((a,b)=>b.score-a.score).slice(0,population);
      if(!best||pool[0].score>best.score)best=pool[0];
      history.push(best.score);
    }
    return {method,best,history,evaluations:count,simulationRuns:count*train.length};
  }
  function benchmark(seeds=[101,102,103,104,105]){
    const train=[11,17,23],test=[41,51,61,71],rows=[];
    for(const seed of seeds){
      const e=search({seed,train}),r=search({seed,train,method:"random"});
      rows.push({seed,evaluations:e.evaluations,simulationsPerMethod:e.simulationRuns,eTrain:e.best.score,rTrain:r.best.score,eTest:evaluate(e.best.g,test),rTest:evaluate(r.best.g,test),fixedTest:evaluate(fixed,test),eGenome:e.best.g,rGenome:r.best.g});
    }
    const mean=k=>rows.reduce((s,r)=>s+r[k],0)/rows.length;
    return {train,test,rows,mean:{eTrain:mean("eTrain"),rTrain:mean("rTrain"),eTest:mean("eTest"),rTest:mean("rTest"),fixedTest:mean("fixedTest")},testWins:rows.filter(r=>r.eTest>r.rTest).length};
  }
  function selfTest(){
    const checks={};
    const assert=(name,ok)=>{checks[name]=Boolean(ok);if(!ok)throw Error("test failed: "+name);};
    const a=simulate(fixed,11),b=simulate(fixed,11);
    assert("determinism",JSON.stringify(a)===JSON.stringify(b));
    const stationary=simulate(fixed,11,{flat:true,drive:false,duration:4,record:true});
    assert("flat unpowered equilibrium",stationary.distance<.01&&!stationary.failed&&stationary.energy===0);
    const powered=simulate(fixed,11,{flat:true,duration:4});
    assert("flat powered motion",powered.distance>2&&powered.energy>0&&!powered.failed);
    const r=rng(9);let g=fixed;for(let i=0;i<1000;i++)g=mutate(g,r);
    assert("mutation bounds",keys.every(k=>g[k]>=bounds[k][0]&&g[k]<=bounds[k][1]));
    const e=search({seed:3,population:8,generations:3}),q=search({seed:3,population:8,generations:3,method:"random"});
    assert("matched budget",e.evaluations===24&&q.evaluations===24&&e.simulationRuns===q.simulationRuns);
    assert("best history monotonic",[e,q].every(s=>s.history.every((v,i)=>i===0||v>=s.history[i-1])));
    const fine=simulate(fixed,11,{dt:1/240}),coarse=simulate(fixed,11,{dt:1/120});
    assert("fixed case timestep score within 10%",Math.abs(fine.score-coarse.score)<=.1*Math.max(1,Math.abs(fine.score)));
    return {checks,stationary,powered,timestep:{coarse:coarse.score,fine:fine.score}};
  }
  return {bounds,fixed,simulate,search,benchmark,selfTest};
}
