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

All three drives are now modelled from first principles: the capstan from cable and radii,
the harmonic from tooth counts and the flexspline cup, the cycloidal from the pin ring and
the output pins. Each model leans on documented approximations (see §8); the datasheet
containers remain available as provenance checks.

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
- `HarmonicDrive` — **derived.** Ratio from tooth counts; stiffness from the flexspline
  cup (Bredt); drag from the wave-generator preload; ripple from the cam runout tolerance;
  loss from bearing and tooth friction. Known approximation: rim flexure is omitted, so
  the stiffness lands above published figures — documented, not hidden.
- `CycloidalDrive` — **derived.** Ratio from the pin count; stiffness from output-pin
  bending plus Hertzian pin contact; backlash from the pin clearance; ripple from two
  tolerances; loss from rolling, bearing and pin sliding. Known approximations: rigid
  disc, cylinder-on-flat contact.
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
COMPARISON.md        derived harmonic-vs-cycloidal findings, side by side
HANDOFF.md           this file
.gitignore

drivelab/
  drives/base.py     THE INTERFACE — implement 5 methods to add a drive
  drives/capstan.py  derived from geometry and material
  drives/harmonic.py derived: tooth counts, flexspline cup, cam tolerance
  drives/cycloidal.py derived: pin ring, output pins, fit tolerances
  drives/ideal.py    lossless references (validation) + datasheet container
  materials.py       isotropic material properties (shared by the geared drives)
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
  capstan_drive.ipynb     the deep dive — start here
  harmonic_drive.ipynb    the derived harmonic study
  cycloidal_drive.ipynb   the derived cycloidal study

foc/                 separate domain, see foc/README.md
tests/
  test_validation.py 38 known-answer checks
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
python tests/test_validation.py    # 38/38
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
| harmonic: `N = z_f/(z_c−z_f)`, Bredt cup+cone, preload, ripple period, derived η | exact |
| cycloidal: `N = z_p−1`, `F_max = 4τ/(z_p r_p)`, pin bending, backlash from clearance, Hertz limit, reflected inertia | exact |
| derived drives conserve energy in their lossless limit | ~1e-9 relative drift |

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
5. **The motor port dropped the transmission-error transformation.** With
   `θ_out = θ_m + e(θ_m)`, the spring torque referred to the motor must carry the factor
   `(1 + de/dθ)` — the cam-follower transformation. Without it, any drive with kinematic
   error quietly injected or absorbed power at the ripple frequency; a lossless harmonic
   drifted ~1e-4 in energy over 1.5 s instead of 1e-9. Found while adding the harmonic,
   and invisible to every drive with `e = 0`.
6. **Ring pins that could not fit.** The first cycloidal reference had 6 mm rollers on a
   31-pin ring at 30 mm pitch radius — overlapping pins, drawn as if possible. The model
   now rejects `r_pin > r_pitch·sin(π/z_p)` outright, and `sized_for` clamps to the
   spacing.

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

Reference configuration: 300 mm arm, 0.5 kg payload, `bldc-frameless-60` motor. **All three
rows are derived models**, each sized to the arm from geometry and material — the capstan and
harmonic by their strength sizing, the cycloidal by the control-stiffness floor (see result 5).

| | capstan 7.7:1 | harmonic 100:1 | cycloidal 30:1 |
|---|---|---|---|
| Π_J (reflected inertia ratio) | **0.18** | **24.5** | 2.26 |
| Ω_n | 51.8 | 94.4 | 69.9 |
| η_f → η_b | 0.985 → 0.985 | 0.778 → 0.714 | 0.957 → 0.955 |
| T1 lost motion [arcmin] | ~0 | ~0 | 4.2 |
| T2 rms error [arcmin] | 46 | 50 | 47 |
| T3 backdrive / mgL (open) | **0.33** | 2.05 | 0.48 |
| T3 backdrive / mgL (shorted) | 0.41 | 5.89 | 1.22 |
| T4 peak transmitted / mgL | **2.0** | **397** | 98 |
| T4 momentum from motor | **15%** | **96%** | 69% |
| T5 resonance/antiresonance ratio | 2.57 | 1.02 | 1.20 |

(For comparison, the datasheet containers give: harmonic Π_J 24.2 / η 0.75→0.67 / T3 3.87 /
T4 262; cycloidal Ω_n 108.7 / η 0.85→0.82 / T3 0.78 / T4 155. The derived models say the
published figures were pessimistic about backdrivability — η_f 0.78 and 0.96 replace the
assumed 0.75 and 0.85.)

### The results

**1. Reflected inertia is the dominant difference, by two orders of magnitude.**
`Π_J = N²J_m/J_l` is 0.18 for the capstan against 24.5 for the 100:1 harmonic — same arm,
same motor. Peak transmitted torque on impact differs ~200×. Nothing on a datasheet says
this, and it follows entirely from `N²`.

**2. The capstan is a mechanical fuse.** Its impact peak landed at *exactly* 1.00× its
rating, because the cable slipped. It physically cannot transmit more than
`2·T_p·r·tanh(μβ/2)`. The cost appears in the same output: ~446 arcmin of permanently
lost registration — the motor encoder no longer knows where the output is. That is a
difference in *kind* between a friction coupling and a toothed one, and no efficiency
figure captures it.

**3. The harmonic's resonance and antiresonance nearly coincide** — separation 1.02
against the capstan's 2.57. With `Π_J ≫ 1` the two features collapse and the usable
control-bandwidth window closes. The separation depends on `Π_J` alone, so it is immune to
the stiffness-approximation caveat in §8.

**4. The capstan's binding constraint is geometric, not tribological.** Not efficiency,
not torque density — **cable bend ratio**. Three quantities pull against each other:
torque wants a small capstan and high pretension; high pretension wants a thick cable; a
thick cable wants a *large* capstan to keep `D/d` healthy. For steel 7×19 on a 90 mm
sector, useful single-stage `N` caps near **8**. This is why real capstan drives are
bulkier than their torque rating suggests and why high-ratio ones are compounded.

**5. The cycloidal is sized by the control loop, not by strength — derived, not asserted.**
A Routh condition on the fixed-controller two-inertia loop puts the stiffness floor at
`Ω_n ≈ 68` for the reference cycloidal; strength-only sizing lands at `Ω_n = 20`; catalog
units sit at ~108. The steel between 20 and 68 is bought purely to close the loop. This is
why cycloidal ratings always look oversized next to their joint loads, and it is derived
live in `cycloidal_drive.ipynb`.

**6. Backdrivability is now predicted for all three drives**, and the predictions are more
backdrivable than the folklore: η_f = 0.78 (harmonic) and 0.96 (cycloidal) replace the
datasheet containers' 0.75 and 0.85, with η_b following as `2 − 1/η_f`. The cycloidal — a
*geared* drive — backdrives at 0.48 mgL open-circuit against the capstan's 0.33.

**7. The harmonic has no stiffness-strength conflict — and that is why it is compact.**
Stiffness and strength both scale with the cup wall; the price is paid elsewhere (preload
drag ∝ rim thickness³, `N²J_m`, backdrivability). The cycloidal likewise scales smoothly,
with no geometric cliff at all — its cost is mass and the stiffness floor.

### Where each wins

- **Capstan** — low reflected inertia, near-symmetric efficiency, zero backlash, overload
  protection by slip. Pays in volume, limited stroke, limited single-stage ratio, and loss
  of registration if it slips. Strongest where backdrivability and impact tolerance
  matter: haptics, force control, collaborative contact.
- **Harmonic** — enormous ratio in a small package, zero backlash. Pays in `N²J_m`, poor
  backdrivability, collapsed bandwidth window. Strongest where precision and ratio
  dominate and the joint will not be backdriven or struck.
- **Cycloidal** — the middle position, with genuine backlash as the distinguishing cost —
  backlash that is itself derived from the output-pin clearance, and deletable by
  preloading. Also the most backdrivable geared drive here, per result 6.

---

## 8. Known limitations — read before quoting any result

1. **The harmonic stiffness omits radial rim flexure.** The derived cup+cone Bredt model
   lands 2.7–5× *above* class-typical published figures for the same size class. The gap
   is reported in the notebook and in `drives/harmonic.py`; conclusions resting on the
   specific `K` inherit it. Conclusions resting on `Π_J`, on the ratios and on the T5
   *separation* (which is `√(1 + 1/Π_J)` regardless of `K`) do not.
2. **Fatigue surrogates.** Both geared drives use a reduced allowable stress plus a safety
   factor in place of a fatigue limit. Real flexsplines and pins die of crack growth; this
   bench has no fatigue model, so the *ratings* are geometry-scaled estimates, not service
   life.
3. **The cycloidal disc is rigid and the pin-ring contact is cylinder-on-flat.** Both are
   documented approximations; the output pins dominate the stiffness anyway.
4. **No failure model.** T4 reports the harmonic transmitting ~43× and the cycloidal ~12×
   their ratings. In reality something breaks first.
5. **The datasheet containers remain, and their old stories remain theirs.** The original
   `harmonic_like` container (K = 7e3, η = 0.75, heavy drag) still saturates its motor on
   T2 — that is a property of those assumed figures, not of the derived model, which
   tracks at 50 arcmin. Keep the two sets of rows labelled.
6. **Catalog entries are representative archetypes**, sized from typical published figures
   for their class, not transcriptions of any manufacturer's datasheet. Every field is a
   documented SI quantity — swapping in real values is a five-minute job and should be
   done before quoting anything about specific hardware.
7. **Single joint, rigid link, one compliance.** No structural flexibility, no thermal
   behaviour, no fatigue, no cable creep dynamics (the `creep_rate` field exists but is
   not simulated), no near-zero-load softening of the harmonic's hysteresis (K1 < K3 is a
   measured property, not a geometric one).
8. **Slip onset resolves one step late.** The slip criterion uses a provisional torque to
   keep the system explicit. Fine for the questions asked; would matter for detailed
   stick-slip work.
9. **`GenericGearedDrive` backlash is a plain deadband** — and so is the cycloidal's
   output-pin clearance. Real backlash has impact dynamics at re-engagement.

---

## 9. Next steps, in priority order

1. **A shell model for harmonic rim flexure.** The single largest remaining approximation.
   A ring-on-foundation or shell strip model for the toothed rim would close most of the
   2.7–5× stiffness gap in §8.1; until then the derived harmonic's `K` is an upper bound.
2. **A second reference arm** where the 100:1 harmonic's ratio is a fair match, so the
   impact comparison stops being "the mismatched ratio" and becomes "the ratio you would
   actually choose".
3. **Two-stage capstan**, since §7.4 says that is what a real high-ratio design is.
4. **Cable creep** — `CableMaterial.creep_rate` is populated but unused. Creep is
   pretension loss, which is torque-capacity loss, which is a genuine field failure mode.
5. **A dedicated comparison notebook** in `notebooks/`, rather than the comparison living
   inside each drive's study (and at the end of the capstan notebook).
6. **Real datasheet values** replacing the catalog archetypes, now that the bench can
   absorb them per-drive.
7. **A bearing-life note for the cycloidal's eccentric orbit** — the bench currently
   reports the orbit's inertia cost (~0.1% of `N²J_m`) but not its bearing fatigue cost,
   which is where the cycloidal's vibration reputation actually lives.

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
- **Ring pins cannot overlap.** `CycloidalDrive` raises unless
  `r_pin ≤ r_pitch·sin(π/z_p)`; `sized_for` clamps. A 6 mm roller on a 31-pin, 30 mm
  ring is impossible geometry, however natural it looks in a parameter list.
- **A strength-sized cycloidal is closed-loop unstable.** With the fixed controller rule,
  the two-inertia Routh condition demands `Ω_n ≳ 68` for the reference cycloidal
  (`Π_J ≈ 2.26`, `ζ_d = 0.02`). Sizing one by strength alone gives `Ω_n ≈ 20` and T2
  blows up at the internal-mode frequency — that is the finding of §7.5, not a bug. Use
  `sized_for(..., min_stiffness=...)`.
- **The notebook embeds its animation as base64.** `to_jshtml()` writes every frame into
  the file, so a committed notebook carries the animation as a blob and a fresh copy lands
  in git history on every re-run. `stride` and `dpi` on the `viz.animate_*` functions
  control this.
- **numpy ≥ 2.0 removed `np.trapz`.** `sim.py` and `bench/metrics.py` use a small
  `_trapz` shim; keep it if you touch those files.
- **The notebooks need the project venv.** `.venv/` (gitignored) was created with
  numpy/scipy/matplotlib/pandas/nbclient/ipykernel/jinja2; execute the notebooks with
  `.venv/bin/python` or install the same into your environment.
- **Python 3.9 target.** No `match`, no `X | Y` unions.

---

## 11. Reproducing everything

```bash
python tests/test_validation.py
```

```bash
jupyter lab notebooks/capstan_drive.ipynb          # the bench, the first study
jupyter lab notebooks/harmonic_drive.ipynb         # the derived harmonic
jupyter lab notebooks/cycloidal_drive.ipynb        # the derived cycloidal
```

Needs numpy, scipy, matplotlib, pandas (and `control` is imported by the environment but
the analytic FRF is hand-rolled, so it is not a hard dependency). No physics engine. The
notebooks were executed with the project venv (`.venv/`, gitignored); see §10.
