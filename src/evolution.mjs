import { createControlledSuspensionLab } from './controlled-suspension.mjs';
// NSGA-II co-evolution for morphology and bounded feedback throttle; reduced-order planar physics.
export function createCoEvolutionLab(physics=createControlledSuspensionLab()){
 const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
 const morphBounds={span:[1.2,3.4],radius:[.22,.55],gear:[2,8],stiffness:[18000,80000],damping:[500,5000],com:[-.08,.08]};
 const controlBounds={bias:[.2,.9],targetSpeed:[1,5],speedKp:[0,.25],pitchKp:[0,1.4],pitchRateKp:[0,.45],slipLimit:[.15,2],slipKp:[0,.4]};
 const morphKeys=Object.keys(morphBounds),controlKeys=Object.keys(controlBounds),baseMorph={...physics.base};
 const baseControl={bias:.62,targetSpeed:3.2,speedKp:.10,pitchKp:.45,pitchRateKp:.12,slipLimit:.55,slipKp:.18};
 function rng(seed){let s=seed>>>0;return ()=>{s=(s+0x6d2b79f5)>>>0;let t=Math.imul(s^(s>>>15),1|s);t^=t+Math.imul(t^(t>>>7),61|t);return ((t^(t>>>14))>>>0)/4294967296;};}
 function repairMorph(g){
  const z={n:clamp(Math.round(g.n),2,4)};
  for(const k of morphKeys)z[k]=clamp(g[k],...morphBounds[k]);
  z.span=Math.max(z.span,2*(z.n-1)*(.22+.025));
  z.radius=Math.min(z.radius,z.span/(2*(z.n-1))-.025);
  z.com=clamp(z.com,-.08,.08);
  return z;
 }
 function repairControl(c){const z={};for(const k of controlKeys)z[k]=clamp(c[k],...controlBounds[k]);return z;}
 function repairGenome(g,mode){
  const morph=mode==="control"?{...baseMorph}:repairMorph(g.morph);
  const control=mode==="morphology"||mode==="fixed"?null:repairControl(g.control??baseControl);
  return {morph,control};
 }
 function sampleMorph(r){return repairMorph({n:2+Math.floor(r()*3),...Object.fromEntries(morphKeys.map(k=>[k,morphBounds[k][0]+r()*(morphBounds[k][1]-morphBounds[k][0])]))});}
 function sampleControl(r){return repairControl(Object.fromEntries(controlKeys.map(k=>[k,controlBounds[k][0]+r()*(controlBounds[k][1]-controlBounds[k][0])] )));}
 function sampleGenome(r,mode){return {morph:mode==="control"?{...baseMorph}:sampleMorph(r),control:mode==="morphology"?null:sampleControl(r)};}
 function policy(c,g){
  if(!c)return null;
  return q=>{
   let slip=0;for(let i=0;i<g.n;i++)slip+=Math.abs(g.radius*q[8+3*i]-q[3]);slip/=g.n;
   const pitch=Math.abs(q[2]),pitchDiverging=q[2]*q[5]>0?Math.abs(q[5]):0;
   return clamp(c.bias+c.speedKp*(c.targetSpeed-q[3])-c.pitchKp*pitch-c.pitchRateKp*pitchDiverging-c.slipKp*Math.max(0,slip-c.slipLimit),0,1);
  };
 }
 const train=[{seed:31,kind:"rough"},{seed:47,kind:"ridge"},{seed:83,kind:"washboard"}];
 const test=[{seed:101,kind:"rough"},{seed:113,kind:"ridge"},{seed:127,kind:"washboard"},{seed:149,kind:"low-grip"}];
 function evaluate(genome,scenarios=train,dt=1/480){
  const g=repairGenome(genome,genome.control===null?"morphology":"joint"),runs=[];
  try{for(const s of scenarios)runs.push(physics.simulate(g.morph,s.seed,{kind:s.kind,dt,duration:6,target:15,controller:policy(g.control,g.morph)}));}
  catch(e){return {feasible:false,error:String(e),fitness:-1e6,objectives:[-1e6,0,-1e6,-1e6,-1e6],runs:[]};}
  const mean=k=>runs.reduce((a,b)=>a+b[k],0)/runs.length;
  const scores=runs.map(x=>x.score),meanScore=mean("score"),variance=scores.reduce((a,x)=>a+(x-meanScore)**2,0)/scores.length,std=Math.sqrt(variance);
  const success=mean("reached"),work=mean("work"),pitch=mean("meanPitch"),distance=mean("distance");
  return {feasible:true,fitness:meanScore+2*success-.5*std,objectives:[meanScore,success,-work/10000,-pitch,-std],meanScore,success,work,pitch,distance,std,meanResidual:mean("relativeEnergyResidual"),runs:runs.map(x=>({score:x.score,reason:x.reason,reached:x.reached,work:x.work,pitch:x.meanPitch,distance:x.distance,residual:x.relativeEnergyResidual,peakSlip:x.peakSlip}))};
 }
 function dominates(a,b){let better=false;for(let i=0;i<a.metrics.objectives.length;i++){if(a.metrics.objectives[i]<b.metrics.objectives[i]-1e-12)return false;if(a.metrics.objectives[i]>b.metrics.objectives[i]+1e-12)better=true;}return better;}
 function sortFronts(pop){
  const fronts=[],dominated=new Map(),count=new Map(),first=[];
  for(const p of pop){const ds=[];let n=0;for(const q of pop){if(p===q)continue;if(dominates(p,q))ds.push(q);else if(dominates(q,p))n++;}dominated.set(p,ds);count.set(p,n);if(n===0){p.rank=0;first.push(p);}}
  fronts.push(first);let i=0;
  while(fronts[i]?.length){const next=[];for(const p of fronts[i])for(const q of dominated.get(p)){count.set(q,count.get(q)-1);if(count.get(q)===0){q.rank=i+1;next.push(q);}}if(next.length)fronts.push(next);i++;}
  for(const front of fronts){
   for(const p of front)p.crowd=0;
   if(front.length<=2){for(const p of front)p.crowd=Infinity;continue;}
   for(let j=0;j<pop[0].metrics.objectives.length;j++){
    const arr=front.slice().sort((a,b)=>a.metrics.objectives[j]-b.metrics.objectives[j]);arr[0].crowd=arr[arr.length-1].crowd=Infinity;
    const lo=arr[0].metrics.objectives[j],hi=arr[arr.length-1].metrics.objectives[j];if(hi-lo<1e-12)continue;
    for(let k=1;k<arr.length-1;k++)if(Number.isFinite(arr[k].crowd))arr[k].crowd+=(arr[k+1].metrics.objectives[j]-arr[k-1].metrics.objectives[j])/(hi-lo);
   }
  }
  return fronts;
 }
 function select(pop,n){const fronts=sortFronts(pop),out=[];for(const f of fronts){if(out.length+f.length<=n)out.push(...f);else{out.push(...f.slice().sort((a,b)=>b.crowd-a.crowd).slice(0,n-out.length));break;}}return out;}
 function tournament(pop,r){const a=pop[Math.floor(r()*pop.length)],b=pop[Math.floor(r()*pop.length)];return a.rank<b.rank?a:b.rank<a.rank?b:a.crowd>b.crowd?a:b.crowd>a.crowd?b:a.metrics.fitness>=b.metrics.fitness?a:b;}
 function cross(a,b,r,mode){
  const child={morph:{},control:null};
  if(mode!=="control"){child.morph.n=r()<.5?a.morph.n:b.morph.n;for(const k of morphKeys){const t=r();child.morph[k]=a.morph[k]*t+b.morph[k]*(1-t);}}
  else child.morph={...baseMorph};
  if(mode!=="morphology"){child.control={};for(const k of controlKeys){const t=r();child.control[k]=a.control[k]*t+b.control[k]*(1-t);}}
  return child;
 }
 function mutate(g,r,mode,scale=.16){
  const z={morph:{...g.morph},control:g.control?{...g.control}:null};
  if(mode!=="control"){
   if(r()<.12)z.morph.n=clamp(z.morph.n+(r()<.5?-1:1),2,4);
   for(const k of morphKeys)if(r()<.55){const [lo,hi]=morphBounds[k];z.morph[k]+=(r()+r()+r()-1.5)*scale*(hi-lo);}
  }
  if(mode!=="morphology"){
   for(const k of controlKeys)if(r()<.6){const [lo,hi]=controlBounds[k];z.control[k]+=(r()+r()+r()-1.5)*scale*(hi-lo);}
  }
  return {morph:mode==="control"?{...baseMorph}:repairMorph(z.morph),control:mode==="morphology"?null:repairControl(z.control)};
 }
 function search({seed=1,method="joint",budget=24,population=8,trainDt=1/480}={}){
  if(!["random","morphology","control","joint"].includes(method)||!Number.isInteger(budget)||budget<population||budget%population!==0)throw Error("invalid search budget");
  const r=rng(seed),all=[];let current=[];
  const addCandidate=genome=>{const g=repairGenome(genome,method),metrics=evaluate(g,train,trainDt),c={genome:g,metrics};all.push(c);return c;};
  if(method==="random"){for(let i=0;i<budget;i++)addCandidate(sampleGenome(r,method));}
  else{
   current=Array.from({length:population},()=>addCandidate(sampleGenome(r,method)));
   for(let done=population;done<budget;done+=population){
    const children=[];for(let i=0;i<population;i++){const a=tournament(current,r),b=tournament(current,r);let child=cross(a.genome,b.genome,r,method);child=mutate(child,r,method);children.push(addCandidate(child));}
    current=select(current.concat(children),population);
   }
  }
  const ranked=all.slice().sort((a,b)=>b.metrics.fitness-a.metrics.fitness),fronts=sortFronts(all);
  return {method,seed,budget,evaluations:all.length,simulationRuns:all.length*train.length,best:ranked[0],paretoSize:fronts[0]?.length??0,front:fronts[0]?.map(x=>({genome:x.genome,metrics:x.metrics}))??[],history:all.filter((x,i)=>i%population===population-1).map((x,i)=>({evaluation:(i+1)*population,bestSoFar:Math.max(...all.slice(0,(i+1)*population).map(y=>y.metrics.fitness))}))};
 }
 function validate(genome,scenarios=test){
  const rows=[];let simulationRuns=0;
  for(const s of scenarios){
   const run=dt=>{simulationRuns++;return physics.simulate(genome.morph,s.seed,{kind:s.kind,dt,duration:6,target:15,controller:policy(genome.control,genome.morph)});};
   const a=run(1/480),b=run(1/960);let selected=b,status="accepted-960",c=null;
   if(a.reason!==b.reason||Math.abs(a.score-b.score)>.05||b.relativeEnergyResidual>.002||b.minDiss< -1e-7){
    c=run(1/1920);
    if(b.reason===c.reason&&Math.abs(b.score-c.score)<=.05&&c.relativeEnergyResidual<=.002&&c.minDiss>=-1e-7){selected=c;status="accepted-1920";}
    else{selected=null;status="unresolved";}
   }
   rows.push({scenario:s,status,score:selected?.score??null,reason:selected?.reason??null,residual960:b.relativeEnergyResidual,residual1920:c?.relativeEnergyResidual??null,delta480960:Math.abs(a.score-b.score),delta9601920:c?Math.abs(b.score-c.score):null,work:selected?.work??null,reached:selected?.reached??false});
  }
  const accepted=rows.filter(x=>x.status!=="unresolved"),meanScore=accepted.length?accepted.reduce((a,b)=>a+b.score,0)/accepted.length:null;
  return {rows,accepted:accepted.length,total:rows.length,meanScore,successRate:accepted.length?accepted.filter(x=>x.reached).length/accepted.length:null,gatePassed:accepted.length===rows.length,simulationRuns};
 }
 function benchmark({seeds=[7,19,41],budget=24,population=8,trainDt=1/480}={}){
  const methods=["random","morphology","control","joint"],rows=[];
  for(const seed of seeds){
   const row={seed};
   for(const method of methods){const result=search({seed,method,budget,population,trainDt}),validation=validate(result.best.genome);row[method]={trainFitness:result.best.metrics.fitness,trainScore:result.best.metrics.meanScore,trainSuccess:result.best.metrics.success,genome:result.best.genome,paretoSize:result.paretoSize,validation};}
   const fixed={morph:{...baseMorph},control:null},fm=evaluate(fixed,train,trainDt),fv=validate(fixed);
   row.fixed={trainFitness:fm.fitness,trainScore:fm.meanScore,trainSuccess:fm.success,genome:fixed,paretoSize:1,validation:fv};
   rows.push(row);
  }
  const labels=[...methods,"fixed"],summary=Object.fromEntries(labels.map(k=>{
   const v=rows.map(r=>r[k]),accepted=v.flatMap(x=>x.validation.rows.filter(z=>z.status!=="unresolved"));
   return [k,{trainScore:v.reduce((s,x)=>s+x.trainScore,0)/v.length,trainSuccess:v.reduce((s,x)=>s+x.trainSuccess,0)/v.length,testScore:accepted.length?accepted.reduce((s,x)=>s+x.score,0)/accepted.length:null,testSuccess:accepted.length?accepted.filter(x=>x.reached).length/accepted.length:null,gatePasses:v.reduce((s,x)=>s+x.validation.accepted,0),gateTotal:v.reduce((s,x)=>s+x.validation.total,0),meanParetoSize:v.reduce((s,x)=>s+x.paretoSize,0)/v.length}];
  }));
  const comparisons={};for(const a of ["joint","morphology","control"])for(const b of ["random","fixed","morphology","control"])if(a!==b){const d=rows.map(r=>r[a].validation.meanScore-r[b].validation.meanScore);comparisons[`${a}Vs${b}`]={wins:d.filter(x=>x>1e-9).length,ties:d.filter(x=>Math.abs(x)<=1e-9).length,losses:d.filter(x=>x< -1e-9).length,mean:d.reduce((s,x)=>s+x,0)/d.length};}
  return {train,test,budget,population,trainDt,searchEvaluations:seeds.length*methods.length*budget,trainingSimulationRuns:seeds.length*methods.length*budget*train.length,fixedBaselineSimulationRuns:seeds.length*train.length,validationSimulationRuns:rows.reduce((s,r)=>s+["random","morphology","control","joint","fixed"].reduce((a,k)=>a+r[k].validation.simulationRuns,0),0),summary,comparisons,rows};
 }
 function selfTest(){
  const checks={},assert=(k,v)=>checks[k]=!!v;
  const g={morph:{...baseMorph},control:{...baseControl}},p=policy(g.control,g.morph);
  assert("controller output bounded",[...Array(21)].every((_,i)=>{const q=Array(15).fill(0);q[3]=i*.4;q[2]=i*.01;q[5]=i*.02;q[8]=2;return p(q)>=0&&p(q)<=1;}));
  assert("zero throttle equals unpowered",Math.abs(physics.simulate(baseMorph,2,{kind:"flat",dt:1/480,duration:2,controller:()=>0}).score-physics.simulate(baseMorph,2,{kind:"flat",dt:1/480,duration:2,drive:false}).score)<1e-10);
  const r=rng(17);let z=sampleGenome(r,"joint");const seen=new Set();
  for(let i=0;i<500;i++){z=repairGenome(mutate(z,r,"joint"),"joint");seen.add(z.morph.n);assert("valid topology",z.morph.span/(z.morph.n-1)>=2*z.morph.radius+.05-1e-9);}
  assert("all wheel counts reachable",seen.size===3);
  const s1=search({seed:3,method:"joint",budget:16,population:8}),s2=search({seed:3,method:"joint",budget:16,population:8});
  assert("deterministic search",JSON.stringify(s1.best.genome)===JSON.stringify(s2.best.genome)&&s1.best.metrics.fitness===s2.best.metrics.fitness);
  const methods=["random","morphology","control","joint"].map(method=>search({seed:4,method,budget:16,population:8}));
  assert("matched budgets",methods.every(x=>x.evaluations===16&&x.simulationRuns===48));
  assert("morphology-only controller fixed",methods[1].best.genome.control===null);
  assert("control-only morphology fixed",JSON.stringify(methods[2].best.genome.morph)===JSON.stringify(baseMorph));
  const fronts=sortFronts([{metrics:{objectives:[1,0]}},{metrics:{objectives:[0,1]}},{metrics:{objectives:[0,0]}}]);
  assert("Pareto sorting",fronts.length===2&&fronts[0].length===2&&fronts[1].length===1);
  const v=validate({morph:baseMorph,control:null},[{seed:5,kind:"flat"}]);
  assert("validation gate result",v.total===1&&v.rows[0].status!=="unresolved");
  return {checks,passed:Object.values(checks).filter(Boolean).length,total:Object.keys(checks).length};
 }
 return {baseMorph,baseControl,train,test,policy,evaluate,search,validate,benchmark,selfTest};
}