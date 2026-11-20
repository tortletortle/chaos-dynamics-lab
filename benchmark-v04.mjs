import {createSuspensionLab} from './src/suspension.mjs';
const lab=createSuspensionLab(),tests=lab.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
console.log(JSON.stringify({tests,benchmark:lab.benchmark()},null,2));
