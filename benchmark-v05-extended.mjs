import { createCoEvolutionLab } from './src/evolution.mjs';
const lab=createCoEvolutionLab();
const tests=lab.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
const benchmark=lab.benchmark({seeds:[7,19,41,59,73],budget:48,population:8,trainDt:1/240});
console.log(JSON.stringify({tests,benchmark},null,2));
