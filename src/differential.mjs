import { createCoEvolutionLab } from './evolution.mjs';
import { createCoEvolutionLab } from './evolution.mjs';
// Mixed-variable differential evolution baseline for joint morphology/controller search.
export function createMixedDE(lab=createCoEvolutionLab()){

 const mb={span:[1.2,3.4],radius:[.22,.55],gear:[2,8],stiffness:[18000,80000],damping:[500,5000],com:[-.08,.08]};
 const cb={bias:[.2,.9],targetSpeed:[1,5],speedKp:[0,.25],pitchKp:[0,1.4],pitchRateKp:[0,.45],slipLimit:[.15,2],slipKp:[0,.4]};
 const mk=Object.keys(mb),ck=Object.keys(cb);
 function rng(seed){let s=seed>>>0;return ()=>{s=(s+0x6d2b79f5)>>>0;let t=Math.imul(s^(s>>>15),1|s);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
 const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
 function repair(g){
  const m={n:clamp(Math.round(g.morph.n),2,4)};
  for(const k of mk)m[k]=clamp(g.morph[k],...mb[k]);
  m.span=Math.max(m.span,2*(m.n-1)*(.22+.025));
  m.radius=Math.min(m.radius,m.span/(2*(m.n-1))-.025);
  const c={};for(const k of ck)c[k]=clamp(g.control[k],...cb[k]);
  return {morph:m,control:c};
 }
 function sample(r){return repair({morph:{n:2+Math.floor(r()*3),...Object.fromEntries(mk.map(k=>[k,mb[k][0]+r()*(mb[k][1]-mb[k][0])])) ,},control:Object.fromEntries(ck.map(k=>[k,cb[k][0]+r()*(cb[k][1]-cb[k][0])]))});}
 function pick3(r,pop,i){const ids=[];while(ids.length<3){const j=Math.floor(r()*pop.length);if(j!==i&&!ids.includes(j))ids.push(j);}return ids.map(j=>pop[j]);}
 function trial(target,a,b,c,r){
  const F=.45+.35*r(),CR=.8,forced=Math.floor(r()*(mk.length+ck.length+1)),m={n:target.genome.morph.n},control={};let idx=0;
  const mutantN=clamp(Math.round(a.genome.morph.n+F*(b.genome.morph.n-c.genome.morph.n)),2,4);
  m.n=(r()<.35||idx===forced)?(r()<.5?mutantN:[a,b,c][Math.floor(r()*3)].genome.morph.n):target.genome.morph.n;idx++;
  for(const k of mk){const use=r()<CR||idx===forced;m[k]=use?a.genome.morph[k]+F*(b.genome.morph[k]-c.genome.morph[k]):target.genome.morph[k];idx++;}
  for(const k of ck){const use=r()<CR||idx===forced;control[k]=use?a.genome.control[k]+F*(b.genome.control[k]-c.genome.control[k]):target.genome.control[k];idx++;}
  return repair({morph:m,control});
 }
 function run({seed=1,budget=48,population=8,trainDt=1/240}={}){
  if(budget<population||budget%population!==0||population<4)throw Error("invalid DE budget");
  const r=rng(seed),all=[];let pop=Array.from({length:population},()=>{const genome=sample(r),metrics=lab.evaluate(genome,lab.train,trainDt),entry={genome,metrics};all.push(entry);return entry;});
  for(let done=population;done<budget;done+=population){
   const next=[];
   for(let i=0;i<population;i++){
    const [a,b,c]=pick3(r,pop,i),genome=trial(pop[i],a,b,c,r),metrics=lab.evaluate(genome,lab.train,trainDt),entry={genome,metrics};all.push(entry);
    next.push(metrics.fitness>=pop[i].metrics.fitness?entry:pop[i]);
   }
   pop=next;
  }
  const best=all.slice().sort((a,b)=>b.metrics.fitness-a.metrics.fitness)[0];
  return {seed,budget,evaluations:all.length,simulationRuns:all.length*lab.train.length,best,finalPopulation:pop.map(x=>({genome:x.genome,fitness:x.metrics.fitness})),history:all.filter((x,i)=>i%population===population-1).map((x,i)=>({evaluations:(i+1)*population,bestSoFar:Math.max(...all.slice(0,(i+1)*population).map(y=>y.metrics.fitness))}))};
 }
 function selfTest(){
  const r=rng(5),g=sample(r),h=sample(r),a=sample(r),b=sample(r),t=trial({genome:g}, {genome:h},{genome:a},{genome:b},r),checks={
   "deterministic rng":rng(9)()===rng(9)(),
   "repair bounds":t.morph.n>=2&&t.morph.n<=4&&t.morph.span/(t.morph.n-1)>=2*t.morph.radius+.05-1e-9,
   "controller bounds":ck.every(k=>t.control[k]>=cb[k][0]&&t.control[k]<=cb[k][1]),
   "continuous bounds":mk.every(k=>t.morph[k]>=mb[k][0]&&t.morph[k]<=mb[k][1]),
   "matched budget":run({seed:2,budget:16,population:8}).evaluations===16
  };
  return {checks,passed:Object.values(checks).filter(Boolean).length,total:Object.keys(checks).length};
 }
 return {run,selfTest};

}
