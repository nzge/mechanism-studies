# mechanism-studies

Study of various mechanisms for future implementation.

**[HANDOFF.md](HANDOFF.md)** — full context: design rationale, results so far, known
limitations, next steps.

## drivelab — a standardized bench for comparing rotary drives

The premise: whatever is inside it, a transmission is a **two-port element** between a
motor and a load, completely described by five behaviours.

| | quantity | where drives differ |
|---|---|---|
| 1 | ratio + kinematic error, `θ_out = θ_m/N + e(θ_m)` | harmonic: 2 cycles/input rev · cycloidal: lobe frequency · capstan: ~0 |
| 2 | compliance, `K(δ)` | capstan: cable EA · harmonic: flexspline windup |
| 3 | loss, `τ_loss(τ_j, ω)` | the asymmetry here *is* backdrivability |
| 4 | reflected inertia, `N²·J_m` | dominates impact behaviour |
| 5 | limits: `τ_max`, stroke, slip | a capstan slips; gears don't |

Every drive implements that interface, so the same tests, the same arm and the same
controller apply to all of them. Adding a transmission means implementing five methods.

### Keeping the comparison honest

A simulation that compares drive technologies is worthless if its conclusions are just
its inputs replayed. Three structural defences:

1. **Parameters are derived where they can be.** Every drive carries a `physics_derived`
   flag. `CapstanDrive` is derived from cable, radii and pretension. `GenericGearedDrive`
   is a datasheet container and says so.
2. **Coupled quantities stay coupled.** Forward and backward efficiency are one knob, not
   two: `η_b = 2 − 1/η_f`, so `η_f = 0.5` is exactly the self-locking boundary.
   Backdrivability is an output of the model.
3. **Scales come from the plant, never the drive.** `τ* = mgL`, `t* = √(J_l/mgL)`. Two
   drives are judged on a load that does not know which drive is attached.

### The dimensionless core

Non-dimensionalizing collapses any drive to six numbers:

```
Π_J = N²J_m/J_l      Ω_n = √(K_out/mgL)      ζ_d      η_f      τ̂_max      e(·)
```

Two transmissions landing on the same six are indistinguishable to the load. Internals
are SI so catalog hardware works directly; analysis is dimensionless.

### The tests

| | test | what it isolates |
|---|---|---|
| T1 | lost motion | input locked, output pushed slowly — hysteresis loop, stiffness |
| T2 | cyclic tracking | PD control, **fixed** bandwidth rule for every drive |
| T3 | backdrive | torque ramp at the output, run open-circuit **and** shorted |
| T4 | impact | swing into a virtual wall referenced to the plant, not the drive |
| T5 | frequency | chirp FRF, plus the analytic resonance/antiresonance pair |

### Layout

```
HANDOFF.md           the long version: rationale, findings, limitations, next steps
drivelab/
  drives/base.py     the two-port interface — implement this to add a drive
  drives/capstan.py  derived from geometry and material
  drives/ideal.py    lossless references + datasheet-driven geared container
  friction.py        LuGre (presliding) and regularized Coulomb
  plant.py  motor.py the lever arm and the motor
  scaling.py         non-dimensionalization, Π groups
  sim.py             the integrator, controllers, external loads
  bench/             T1–T5 and metric extraction
  catalog/           motors, cables, arms, named configurations
  viz.py             plots (CVD-validated palette) and animation
notebooks/           one notebook per drive study
foc/                 separate domain — motor control, not transmissions
tests/               known-answer validation of the bench itself
```

One notebook per drive rather than a directory each: the studies are single files, and
seven directories holding one file apiece was structure without information. `foc/` keeps
its own directory because it is a genuinely different problem — see [foc/README.md](foc/README.md).

### Validation

`python tests/test_validation.py` — 15 known-answer checks. A rigid lossless drive must
reduce to a pendulum of inertia `J_l + N²J_m`; a lossless drive must conserve energy; the
two-inertia resonance must land at `√(K(1/J_refl + 1/J_l))`; the capstan must match its
closed forms. A bench that cannot reproduce the cases where the answer is already known
has no business comparing the ones where it isn't.

### Running

Needs numpy, scipy, matplotlib, pandas. Start with
[notebooks/capstan_drive.ipynb](notebooks/capstan_drive.ipynb), then
[HANDOFF.md](HANDOFF.md) for the reasoning, the findings and the known gaps.

```bash
python tests/test_validation.py
```

### What this cannot tell you

Nothing about fatigue life, thermal behaviour, tolerance stack-up, cost or
manufacturability. It is a rigid-body-plus-one-compliance model of a single joint, with
no failure model — T4 will happily report a harmonic drive transmitting 55× its rating.
Harmonic and cycloidal parameters are currently datasheet-shaped rather than derived, so
conclusions resting on their specific values inherit those figures' accuracy. Conclusions
resting on `N²J_m` are robust, because that term is geometric.
