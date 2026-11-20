import { readFileSync } from 'node:fs';
import { createNumericsLab } from './src/numerics.mjs';
const prior=JSON.parse(readFileSync(new URL('./experiments/v0.2.json',import.meta.url),'utf8')).benchmark;
const lab=createNumericsLab();
const tests=lab.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
console.log(JSON.stringify({tests,audit:lab.audit(prior)},null,2));
