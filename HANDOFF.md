# Handoff — drivelab

Everything needed to pick this up cold: what it is, why it is built this way, what it
has actually found, what is known to be wrong with it, and where to go next.

Companion to [README.md](README.md), which is the short version. This one is the long
version, written for whoever touches the code next — including future-you.

---

## 1. What this is

A simulation bench for comparing rotary drive transmissions — capstan, harmonic,
cycloidal — under identical, standardized tests, so that claims like "capstans are more
backdrivable" become measurements rather than received wisdom.

The first subject is the cable capstan drive, modelled from first principles.

### The question it was built to answer

Given a joint to design, which transmission should go in it, and **why** — not by
reputation, but by a chain of reasoning from geometry and material to behaviour under
load.

### The question it cannot answer

Anything about fatigue life, heat, tolerance stack-up, cost, or manufacturability. It is
a rigid-body-plus-one-compliance model of a single joint. It will not tell you when your
cable breaks.

---

## 2. The central design decision, and why

**Everything hangs on one idea: a transmission is a two-port element.** Whatever is
inside it, it sits between a motor and a load and is completely described by five
behaviours.

| | quantity | where drives differ |
|---|---|---|
| 1 | ratio + kinematic error, `θ_out = θ_m/N + e(θ_m)` | harmonic: 2 cycles/input rev · cycloidal: lobe frequency · capstan: ~0 |
| 2 | compliance, `K(δ)` | capstan: cable EA · harmonic: flexspline windup |
| 3 | loss, `τ_loss(τ_j, ω)` | the asymmetry here *is* backdrivability |
| 4 | reflected inertia, `N²·J_m` | dominates impact behaviour |
| 5 | limits: `τ_max`, stroke, slip | a capstan slips; gears don't |

That is the whole of `drivelab/drives/base.py`. Adding a transmission means implementing
those five and nothing else — no test, metric or plot ever needs to know which drive it
is looking at.

### Why this abstraction and not a multibody engine

PyBullet, MuJoCo and Drake model a transmission as an ideal ratio. Every phenomenon this
bench cares about — cable slip, load-dependent efficiency, tooth-mesh error, presliding
friction — would have to be written as custom code anyway, and then fought through the
engine's solver. A hand-rolled stiff ODE in scipy is more transparent, more accurate for
these questions, and debuggable. This was a deliberate choice, not an omission.

---

## 3. How the comparison is kept honest

This is the part most worth preserving if the code is ever rewritten. A simulation
comparing drive technologies is worthless if its conclusions are its inputs replayed
with axes drawn on them. Four structural defences:

### 3.1 Provenance is tracked per model

Every drive carries a `physics_derived` flag.

- `CapstanDrive` — **derived.** Ratio, stiffness, torque, stroke and drag all follow from
  cable, radii and pretension.
- `GenericGearedDrive` — **not derived.** An honest container for published
  harmonic/cycloidal figures. Useful as a bench subject; proves nothing about mechanism.

A plot mixing the two is still valid — it just answers a different question. Keeping the
distinction visible is the point.

### 3.2 Coupled quantities stay coupled

Forward and backward efficiency are **one knob, not two**. Under a loss model where
dissipation is proportional to transmitted torque, with `c = 1/η_f − 1`:

```
η_b = 2 − 1/η_f
```

So `η_f = 0.8 → η_b = 0.75`; `η_f = 0.6 → 0.33`; and `η_f = 0.5` is *exactly* the
self-locking boundary. Backdrivability is therefore an **output** of the model. You
cannot tune it to get the answer you want. (Same relation that governs lead screws.)

### 3.3 Scales come from the plant, never the drive

`τ* = mgL`, `t* = √(J_l/mgL)`. Two drives are judged on a load that does not know which
drive is attached to it. Non-dimensionalizing collapses any transmission to six numbers:

```
Π_J = N²J_m/J_l     Ω_n = √(K_out/mgL)     ζ_d     η_f     τ̂_max     e(·)
```

Two transmissions landing on the same six are indistinguishable to the load. That is the
claim the bench exists to test.

### 3.4 The tests are fixed, never tuned per drive

- **The controller is the same for every drive.** Same `alpha` bandwidth target. A drive
  that cannot support it rings or goes unstable — and that *is* the finding. Tuning each
  drive to its own limit would hide exactly the difference being measured.
- **The impact wall is referenced to the plant**, `k_wall = Ω²·mgL`, so every drive hits
  the identical obstacle. (This was a bug once — see §6.)

---

## 4. Layout

```
README.md            short version
HANDOFF.md           this file
.gitignore

drivelab/
  drives/base.py     THE INTERFACE — implement 5 methods to add a drive
  drives/capstan.py  derived from geometry and material
  drives/ideal.py    lossless references (validation) + datasheet container
  friction.py        LuGre (presliding) and regularized Coulomb
  plant.py           the lever arm — the common test load
  motor.py           inertia, torque ceiling, kt²/R short-circuit damping
  scaling.py         non-dimensionalization, Π groups
  config.py          binds motor+drive+plant; feasibility checks
  sim.py             integrator, controllers, external loads
  bench/tests.py     T1–T5
  bench/metrics.py   metric extraction
  catalog/           motors, cables, arms, named configurations
  viz.py             plots (CVD-validated palette) and animation

notebooks/
  capstan_drive.ipynb    the deep dive — start here
  cycloidal_drive.ipynb  empty placeholder

foc/                 separate domain, see foc/README.md
tests/
  test_validation.py 15 known-answer checks
```

### Why `notebooks/` and not a folder per drive

Each drive study is one notebook. Seven directories holding one file each was structure
without information. FOC keeps its own directory because it is a genuinely separate
problem — different models, different time scales (tens of kHz vs tens of Hz), different
rigs.

### The state vector

Six states, fixed layout, same for every drive:

```
[ θ_m_out, ω_m_out, θ_l, ω_l, θ_slip, z ]
```

Everything is **output-referred** (`θ_m_out = θ_m/N`), so the reduction ratio never
appears in the equations of motion — it is folded into reflected inertia and into how
motor torque is reported. That is what makes two drives of different ratio directly
comparable on the same axes.

Drives that cannot slip return zero for `θ_slip`. One unused state buys a much simpler
integrator.

---

## 5. The tests

| | test | what it isolates | the gotcha it avoids |
|---|---|---|---|
| T1 | lost motion | input locked, output pushed slowly | lock bandwidth must far exceed the drive's resonance, or you measure the lock |
| T2 | cyclic tracking | PD control, fixed bandwidth rule | no per-drive tuning |
| T3 | backdrive | torque ramp, motor unpowered | run **both** open-circuit and shorted |
| T4 | impact | swing into a virtual wall | wall referenced to plant, not drive |
| T5 | frequency | chirp FRF + analytic resonance pair | measured *and* analytic, so nonlinearity shows |

**T3 must be reported both ways.** The mechanism backdrives; the *motor* may not. For a
low-ratio drive the `kt²/R` term is frequently the dominant resistance — quoting only the
open-circuit number is how backdrivability claims get overstated.

### Runtime

A full suite is roughly 30–50 s per drive on this machine. The system is genuinely stiff
(a steel-cable capstan is ~10⁴× stiffer than the load dynamics), so the solver is `Radau`
by default. An explicit solver will either crawl or lie.

---

## 6. Validation — and the bugs it caught

```bash
python tests/test_validation.py    # 15/15
```

| check | result |
|---|---|
| rigid lossless drive == pendulum of `J_l + N²J_m` | 0.003–0.010% period error |
| lossless drive conserves energy | 2.8e-14 relative drift |
| two-inertia resonance at `√(K(1/J_refl + 1/J_l))` | 0.04% error |
| `η_b = 2 − 1/η_f`, self-locking at 0.5 | exact |
| capstan closed forms, slip-before-slack | exact |
| wrap saturation: 3→10 wraps gains <5% | +4.7% |
| shorted motor resists backdriving more than free | confirmed |

A bench that cannot reproduce cases where the answer is already known has no business
comparing the ones where it isn't.

### Bugs found during development — all would have produced plausible-looking plots

1. **The impact wall was scaled by each drive's own stiffness.** A fairness bug: every
   drive hit a *different* wall, so the comparison was partly measuring the test rig.
   Now referenced to the plant.
2. **LuGre micro-damping was sized ~40× too high.** Sizing `σ1 = 2√(σ0·J)` with the load
   inertia — the textbook form for bristle critical damping — produced a coefficient far
   above the Coulomb level, so every motion reversal dissipated as though through a heavy
   dashpot. Now tied to the friction level and Stribeck velocity.
3. **The slip limit capped only the elastic term.** A friction coupling cannot transmit
   more than its grip limit however fast it is deformed, but the damper term was
   unbounded, so impacts pushed arbitrary torque through a "slipping" capstan.
4. **The torque channel reported commanded, not delivered, torque** — ignoring motor
   saturation, and every derived quantity inherited the error.

### A metric that was removed on purpose

T2 reports **no simulated efficiency number**. Every formulation was broken:
`energy_out/energy_in` is ~0/~0 over a closed cycle; `1 − loss/in` explodes when gravity
feedforward drives net motor work toward zero; and `|τ·ω|` throughput counts a saturated
drive's arm sloshing on its own compliance as useful work — flattering precisely the
drives that are *failing* the test. (The harmonic scored 0.977 this way while not
tracking at all.)

What it reports instead: raw energies, the well-defined quasi-static efficiency at peak
load, and honest `saturated` / `tracking_ok` flags.

**The lesson generalizes: if a metric can be gamed by oscillation, it will be.**

---

## 7. What the bench has found so far

Reference configuration: 300 mm arm, 0.5 kg payload, `bldc-frameless-60` motor.

| | capstan 7.7:1 | harmonic 100:1 | cycloidal 30:1 |
|---|---|---|---|
| Π_J (reflected inertia ratio) | **0.18** | **24.2** | 2.18 |
| Ω_n | 51.8 | 64.3 | 108.7 |
| η_f → η_b | 0.985 → 0.985 | 0.75 → 0.67 | 0.85 → 0.82 |
| T1 lost motion [arcmin] | ~0 | ~0 | 1.5 |
| T3 backdrive / mgL (open) | **0.33** | 3.87 | 0.78 |
| T3 backdrive / mgL (shorted) | 0.41 | 6.90 | 1.56 |
| T4 peak transmitted / mgL | **2.0** | **262** | 155 |
| T4 momentum from motor | **15%** | **96%** | 69% |
| T5 resonance/antiresonance ratio | 2.57 | 1.02 | 1.21 |

### The four results

**1. Reflected inertia is the dominant difference, by two orders of magnitude.**
`Π_J = N²J_m/J_l` is 0.18 for the capstan against 24.2 for the 100:1 harmonic — same arm,
same motor. Peak transmitted torque on impact differs ~130×. Nothing on a datasheet says
this, and it follows entirely from `N²`.

**2. The capstan is a mechanical fuse.** Its impact peak landed at *exactly* 1.00× its
rating, because the cable slipped. It physically cannot transmit more than
`2·T_p·r·tanh(μβ/2)`. The cost appears in the same output: ~446 arcmin of permanently
lost registration — the motor encoder no longer knows where the output is. That is a
difference in *kind* between a friction coupling and a toothed one, and no efficiency
figure captures it.

**3. The harmonic's resonance and antiresonance nearly coincide** — separation 1.02
against the capstan's 2.57. With `Π_J ≫ 1` the two features collapse and the usable
control-bandwidth window closes. Invisible in any static specification.

**4. The capstan's binding constraint is geometric, not tribological.** Not efficiency,
not torque density — **cable bend ratio**. Three quantities pull against each other:
torque wants a small capstan and high pretension; high pretension wants a thick cable; a
thick cable wants a *large* capstan to keep `D/d` healthy. For steel 7×19 on a 90 mm
sector, useful single-stage `N` caps near **8**. This is why real capstan drives are
bulkier than their torque rating suggests and why high-ratio ones are compounded.

### Where each wins

- **Capstan** — low reflected inertia, near-symmetric efficiency, zero backlash, overload
  protection by slip. Pays in volume, limited stroke, limited single-stage ratio, and loss
  of registration if it slips. Strongest where backdrivability and impact tolerance
  matter: haptics, force control, collaborative contact.
- **Harmonic** — enormous ratio in a small package, zero backlash. Pays in `N²J_m`, poor
  backdrivability, collapsed bandwidth window. Strongest where precision and ratio
  dominate and the joint will not be backdriven or struck.
- **Cycloidal** — the middle position, with genuine backlash as the distinguishing cost.

---

## 8. Known limitations — read before quoting any result

1. **Harmonic and cycloidal are datasheet containers, not derived models.** Conclusions
   resting on their *specific* values inherit those figures' accuracy. Conclusions resting
   on `N²J_m` are robust, because that term is geometric. **This is the single biggest
   gap.**
2. **The harmonic reference config saturates its motor** on T2 (`tracking_ok = False`).
   That is itself a finding — a 100:1 harmonic is badly matched to a light desktop arm —
   but the tracking column is not a tracking comparison. Either resize it or add an arm
   where it is a fair fight.
3. **No failure model.** T4 reports the harmonic transmitting ~55× its rating. In reality
   something breaks first.
4. **Catalog entries are representative archetypes**, sized from typical published figures
   for their class, not transcriptions of any manufacturer's datasheet. Every field is a
   documented SI quantity — swapping in real values is a five-minute job and should be
   done before quoting anything about specific hardware.
5. **Single joint, rigid link, one compliance.** No structural flexibility, no thermal
   behaviour, no fatigue, no cable creep dynamics (the `creep_rate` field exists but is
   not simulated).
6. **Slip onset resolves one step late.** The slip criterion uses a provisional torque to
   keep the system explicit. Fine for the questions asked; would matter for detailed
   stick-slip work.
7. **`GenericGearedDrive` backlash is a plain deadband.** Real backlash has impact
   dynamics at re-engagement.

---

## 9. Next steps, in priority order

1. **Derived harmonic and cycloidal models.** The big one. Tooth-mesh kinematics for
   transmission error, flexspline compliance from geometry, load-dependent mesh friction.
   Then the comparison is derived-to-derived and can speak to *mechanism* rather than to
   specification. The interface is five methods.
2. **A second reference arm** where the 100:1 harmonic is fairly matched, so T2 compares
   tracking rather than saturation.
3. **Two-stage capstan**, since §7.4 says that is what a real high-ratio design is.
4. **Cable creep** — `CableMaterial.creep_rate` is populated but unused. Creep is
   pretension loss, which is torque-capacity loss, which is a genuine field failure mode.
5. **A dedicated comparison notebook** in `notebooks/`, rather than the comparison living
   at the end of the capstan notebook.
6. Real datasheet values replacing the catalog archetypes.

---

## 10. Gotchas for whoever works on this next

- **Don't start a test from zero windup.** Use `sim.quasi_static_state()`. Starting
  unwound rings the drive's internal mode at full amplitude, and that transient is an
  artifact of the initial condition — it shows up as spurious overshoot in T4 and
  spurious energy in T2.
- **Watch solver cost when choosing stiffness.** An early validation test used `K = 1e8`
  and asked Radau to resolve a 50 kHz mode for 6 seconds at `rtol=1e-10`. It hung. Size
  the internal mode deliberately (the tests target ~60× the pendulum frequency) rather
  than picking a large number and hoping.
- **A series damper does not damp the rigid-body mode.** Both inertias turn together, so
  relative motion is zero. This is useful: it kills the fast mode for cheap integration
  without touching the pendulum period being measured.
- **Damping reference inertia must match between `damping_out()` and `damping_ratio()`**,
  or the ζ you ask for is not the ζ you get — and one drive quietly gets extra damping in
  every transient test.
- **Check `cfg.feasibility()` before believing a result.** It catches a motor that cannot
  hold the arm up, a drive under-rated for the load, a stroke limit shorter than the
  commanded motion, and a cable bend ratio below its minimum. All of these otherwise
  produce entirely plausible-looking plots.
- **The notebook embeds its animation as base64.** `to_jshtml()` writes every frame into
  the file, so a committed notebook carries the animation as a blob and a fresh copy lands
  in git history on every re-run. `stride` and `dpi` on `viz.animate_capstan` control this.
- **Python 3.9 target.** No `match`, no `X | Y` unions.

---

## 11. Reproducing everything

```bash
python tests/test_validation.py
```

```bash
jupyter lab notebooks/capstan_drive.ipynb
```

Needs numpy, scipy, matplotlib, pandas (and `control` is imported by the environment but
the analytic FRF is hand-rolled, so it is not a hard dependency). No physics engine.
