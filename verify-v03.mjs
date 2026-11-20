import { readFileSync } from 'node:fs';
import { createVerificationGate } from './src/verification.mjs';
const a=JSON.parse(readFileSync(new URL('./experiments/v0.3.json',import.meta.url),'utf8')).audit;
const g=createVerificationGate(),tests=g.selfTest();
if(tests.passed!==tests.total)throw Error(JSON.stringify(tests));
console.log(JSON.stringify({tests,report:g.auditGate(a.details)},null,2));
