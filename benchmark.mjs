import { createLab } from './src/lab.mjs';
const lab = createLab();
console.log(JSON.stringify({tests:lab.selfTest(),benchmark:lab.benchmark()},null,2));
