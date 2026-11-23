import {createCoEvolutionLab} from './src/evolution.mjs';
const lab=createCoEvolutionLab(),tests=lab.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
console.log(JSON.stringify({tests,benchmark:lab.benchmark()},null,2));
