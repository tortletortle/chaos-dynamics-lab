import { createMorphLab } from './src/morph.mjs';
const lab=createMorphLab();
const tests=lab.selfTest();
if(tests.passed!==tests.total) throw new Error(JSON.stringify(tests));
console.log(JSON.stringify({tests,benchmark:lab.benchmark()},null,2));
