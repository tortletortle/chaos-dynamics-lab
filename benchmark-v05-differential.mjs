import { createCoEvolutionLab } from './src/evolution.mjs';
import { createMixedDE } from './src/differential.mjs';
const lab=createCoEvolutionLab(),de=createMixedDE(lab),tests=de.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
const rows=[7,19,41,59,73].map(seed=>de.run({seed,budget:48,population:8,trainDt:1/240}));
console.log(JSON.stringify({tests,rows},null,2));
