// Numerical consistency gate; not a guarantee of physical validity.
export function createVerificationGate(){
  const scoreTol=.05;
  const valid=x=>x&&Number.isFinite(x.score)&&typeof x.reason==="string"&&Number.isInteger(x.limited)&&x.limited>=0;
  const agrees=(a,b)=>valid(a)&&valid(b)&&a.limited===0&&b.limited===0&&a.reason===b.reason&&Math.abs(a.score-b.score)<=scoreTol;
  function decide({a120,a240,ref,ref2}){
    if(agrees(a120,a240))return {status:"adaptive-consistent",selected:a240,reason:"two adaptive resolutions agree"};
    if(!ref||!ref2)return {status:"needs-refinement",selected:null,reason:"adaptive event, score, or tolerance-floor disagreement"};
    if(agrees(ref,ref2))return {status:"reference-consistent",selected:ref2,reason:"two finer fixed-step solutions agree"};
    return {status:"unresolved",selected:null,reason:"reference disagreement, invalid result, or tolerance floor"};
  }
  function verifyCase(lab,g,seed,options={}){
    const cfg={...options,record:false};
    const a120=lab.simulate(g,seed,{...cfg,mode:"adaptive",dt:1/120}),a240=lab.simulate(g,seed,{...cfg,mode:"adaptive",dt:1/240});
    const first=decide({a120,a240});if(first.status!=="needs-refinement")return {...first,a120,a240};
    const ref=lab.simulate(g,seed,{...cfg,mode:"midpoint",dt:1/960}),ref2=lab.simulate(g,seed,{...cfg,mode:"midpoint",dt:1/1920});return {...decide({a120,a240,ref,ref2}),a120,a240,ref,ref2};
  }
  function auditGate(cases){
    const rows=cases.map(c=>{const d=decide(c);return {searchSeed:c.searchSeed,method:c.method,terrainSeed:c.terrainSeed,status:d.status,selectedScore:d.selected?.score??null,referenceScore:c.ref2.score,absoluteDelta:d.selected?Math.abs(d.selected.score-c.ref2.score):null,eventAgreement:d.selected?d.selected.reason===c.ref2.reason:null,forceCalls:c.a120.forceCalls+c.a240.forceCalls+(d.status==="adaptive-consistent"?0:c.ref.forceCalls+c.ref2.forceCalls)};});
    const good=rows.filter(x=>x.absoluteDelta!==null);return {scoreTolerance:scoreTol,total:rows.length,statusCounts:Object.fromEntries(["adaptive-consistent","reference-consistent","needs-refinement","unresolved"].map(s=>[s,rows.filter(x=>x.status===s).length])),maxDelta:Math.max(...good.map(x=>x.absoluteDelta)),meanDelta:good.reduce((s,x)=>s+x.absoluteDelta,0)/good.length,eventDisagreements:good.filter(x=>!x.eventAgreement).length,meanForceCalls:rows.reduce((s,x)=>s+x.forceCalls,0)/rows.length,rows};
  }
  function selfTest(){const x={score:1,reason:"goal",limited:0},y={...x,score:1.01},z={...x,reason:"timeout"},floor={...x,limited:1};
    const tests={agree:decide({a120:x,a240:y}).status==="adaptive-consistent",eventMismatch:decide({a120:x,a240:z}).status==="needs-refinement",floor:decide({a120:x,a240:floor}).status==="needs-refinement",refinement:decide({a120:x,a240:z,ref:y,ref2:x}).status==="reference-consistent",unresolved:decide({a120:x,a240:z,ref:x,ref2:z}).status==="unresolved",nonfiniteRejected:decide({a120:x,a240:{...x,score:NaN}}).status==="needs-refinement"};
    // Exercise orchestration using a deterministic simulator stub; real solver audit uses recorded runs.
    let calls=0;const stub={simulate(){calls++;return calls===2?z:x;}};
    tests.refineOnLiveInterface=verifyCase(stub,{},1).status==="reference-consistent"&&calls===4;
    return {tests,passed:Object.values(tests).filter(Boolean).length,total:Object.keys(tests).length};
  }
  return {decide,verifyCase,auditGate,selfTest};
}
