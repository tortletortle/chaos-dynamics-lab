# Chaos Dynamics Lab

AI-assisted evolutionary vehicle design in a reproducible, reduced-order physics sandbox.

## Current scope: V0.5

The prototype now includes:

- A planar chassis model with pitch, independent vertical wheel travel, wheel spin, spring/damper suspension and a simplified slip-dependent tire force.
- A bounded feedback-throttle interface. The controller observes chassis speed, pitch, pitch rate and estimated wheel slip.
- Parameterized vehicle morphology: 2–4 wheel stations, wheel radius, wheelbase, gearing, suspension stiffness/damping and center-of-mass offset.
- NSGA-II search with morphology-only, controller-only and joint morphology/controller modes.
- Matched-budget random-search baseline.
- Held-out terrain validation at 480/960 Hz, with 1920 Hz fallback when the two resolutions disagree.

This is a research prototype, not an industrial-grade vehicle simulator. It is a 2D longitudinal/vertical reduced-order model: it does not yet include steering/lateral dynamics, full rigid-body contact, explicit linkage geometry, tire deformation/temperature, or independently actuated wheels.

## Run locally

Requires Node.js with native ES module support; there are no third-party runtime dependencies.

```bash
node benchmark-v05.mjs
node benchmark-v05-extended.mjs
```

The extended run uses five search seeds, 48 candidate evaluations per method and a 240 Hz training step. It then validates selected candidates on four held-out terrain cases using 480/960 Hz comparisons and a 1920 Hz fallback.

## V0.5 extended experiment

Recorded in [experiments/v0.5-extended.json](experiments/v0.5-extended.json).

- 5 independent search seeds.
- 4 search methods: random, morphology-only NSGA-II, controller-only NSGA-II, joint NSGA-II.
- 48 candidate evaluations per method and seed: 960 search evaluations total.
- 2,880 training simulations, plus 15 fixed-baseline training simulations.
- 202 held-out validation simulations.
- All 20 held-out cases per method passed the numerical consistency gate; one random-search case and one joint-search case required 1920 Hz fallback.

Mean held-out scores:

| Method | Mean score | Goal rate |
|---|---:|---:|
| Random search | 17.29 | 75% |
| Morphology-only | 16.24 | 80% |
| Controller-only | 14.08 | 40% |
| Joint co-evolution | 16.08 | 70% |
| Fixed baseline | 13.13 | 50% |

The joint method did **not** outperform random search in this experiment (mean paired difference: -1.20 across five seeds). This is a negative result, not evidence that co-evolution is superior. The sample is small and the controller is currently a single shared throttle policy, not an RL controller.

## Reproducibility and limitations

- Training and held-out terrain seeds are fixed in source.
- Every search method receives the same candidate-evaluation budget.
- The 480/960/1920 Hz gate checks score and terminal-event agreement plus the fine-run energy-residual and dissipation checks. Numerical agreement does not prove real-world physical accuracy.
- The tire, suspension and chassis equations remain simplified approximations and have not been calibrated against a physical test rig.
- No PPO/SAC agent, 3D engine, deformable terrain, steering, or hardware validation is implemented yet.

See [docs/V0.5-REPORT.md](docs/V0.5-REPORT.md) for the experiment interpretation and next research priorities.
