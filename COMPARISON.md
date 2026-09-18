# Comparison Findings — Harmonic vs Cycloidal (derived models)

The two geared drives, side by side, with every number *derived* from geometry, material
and tolerances rather than quoted from a datasheet. The capstan row is included as the
reference line: it is the other end of the spectrum, and the two drives are best read
against it.

Reference conditions, identical for all rows: 300 mm arm, 0.5 kg payload,
`bldc-frameless-60` motor, the bench's five fixed tests (T1–T5), the same controller rule
for every drive. Full derivations live in `notebooks/harmonic_drive.ipynb` and
`notebooks/cycloidal_drive.ipynb`; validation in `tests/test_validation.py` (38/38).

---

## 1. The numbers

### As the load sees them (the six dimensionless groups)

| | capstan 7.7:1 | harmonic 100:1 | cycloidal 30:1 |
|---|---|---|---|
| Π_J — reflected inertia ratio `N²J_m/J_l` | **0.18** | **24.5** | 2.26 |
| Ω_n — stiffness `√(K_out/mgL)` | 51.8 | 94.4 | 69.9 |
| ζ_d — structural damping ratio | 0.02 | 0.02 | 0.02 |
| η_f → η_b (derived) | 0.985 → 0.985 | 0.778 → 0.714 | 0.957 → 0.955 |
| τ̂_max — torque rating / mgL | 2.0 | 9.2 | 8.1 |
| e(·) — ripple | ~0 | 0.78′, 2/input rev | 1.15′, mixed |

### The five tests

| test | metric | capstan | harmonic (derived) | cycloidal (derived) |
|---|---|---|---|---|
| T1 | lost motion [arcmin] | ~0 | ~0 | **4.2** |
| T1 | measured stiffness / derived | ✓ recovers `2EA/L·R²` | ✓ recovers Bredt | deadband-contaminated |
| T2 | tracking rms error [arcmin] | 46 | 50 | 47 |
| T2 | energy lost per cycle / mgL | 0.09 | **3.23** | 0.26 |
| T2 | saturated / tracking ok | no / yes | no / yes | no / yes |
| T3 | backdrive, open circuit / mgL | **0.33** | 2.05 | 0.48 |
| T3 | backdrive, shorted / mgL | **0.41** | 5.89 | 1.22 |
| T4 | peak transmitted / mgL | **2.0** (=1.0× rating) | **397** (=43× rating) | 98 (=12× rating) |
| T4 | momentum from the motor | 15% | 96% | 69% |
| T5 | antiresonance [Hz] | 48.2 | 87.8 | 65.1 |
| T5 | resonance [Hz] | 123.8 | 89.6 | 78.1 |
| T5 | resonance/antiresonance ratio | 2.57 | **1.02** | 1.20 |

---

## 2. Reading the six numbers

What each of the six dimensionless groups means, where its value comes from, and where it
shows up. These six are the *entire* drive as the load experiences it — two drives landing
on the same six are indistinguishable to the arm.

### 1 · Π_J = N²J_m / J_l — reflected inertia ratio

**What it is.** Seen from the output side, the rotor does not feel like `J_m`; it feels
like `N²J_m`, because the motor spins N times faster (its kinetic energy is
`½J_m(N·ω_out)² = ½(N²J_m)ω_out²`). Π_J is how much the motor — through the ratio —
outweighs the arm's own inertia.

**Where the value comes from.** `N` is a counting argument (100 vs 30); `J_m` is the
catalog rotor value; `J_l` is arm geometry (point mass `mL²` + rod `mL²/3`). The drive's
*own* inertia is added on top (harmonic: flexspline + `N²·J_wg` ≈ 0.014 kg·m²; cycloidal:
`N²·m_d·e²` orbit term + cam ≈ 0.004 kg·m² — both small next to the motor's 1.2 and
0.108 kg·m²).

**Where it bites.** Two places, both invisible on a spec sheet:
- **T4 impact** — the motor contributes `Π_J/(1+Π_J)` of the momentum arriving at the
  wall: 96% (harmonic), 69% (cycloidal), 15% (capstan). It *is* the impact test.
- **T5** — the resonance/antiresonance separation is exactly `√(1 + 1/Π_J)`: 1.02
  (harmonic), 1.20 (cycloidal), 2.57 (capstan). With Π_J ≫ 1 the two features collapse
  and the usable control window closes.

This is the dominant difference between the drives — two orders of magnitude — and it
follows entirely from `N²`.

### 2 · Ω_n = √(K_out / mgL) — stiffness, in plant units

**What it is.** The drive's torsional spring constant `K_out`, normalized by the plant's
torque scale: stiffness in units of "gravity loads per radian". Ω_n is also the frequency
(in plant time units) at which the arm would bounce on the drive's spring with the motor
locked — the drive's antiresonance, the notch a controller cannot push through.

**Where the value comes from.** `K_out` is the biggest derived quantity in each model,
and the physics differs per mechanism:

- **harmonic** — two *exact* thin-tube (Bredt) torsion terms in series:
  `K_cup = 2πGr³t/L`, `K_cone = 4πGt·sinα/(1/a²−1/b²)`. Sized to the arm (thinnest cup
  that survives 2× gravity) → 15.1 kN·m/rad; a size-typical 0.5 mm cup → 42 kN·m/rad.
- **cycloidal** — output pins as cantilevers in bending, `n·3EI/L³·R_out²`, in series
  with the pin-ring Hertzian line contact. Sized to the *control floor* (below) →
  8.3 kN·m/rad.
- **capstan** (reference) — `2(EA/L)R²` from the cable → 4.5 kN·m/rad.

Dividing by `mgL = 1.69 N·m` and taking the square root gives 94.4, 69.9, 51.8.

**Where it bites.** T5's antiresonance frequency (`√(K/J_l)`); T2's tracking stiffness;
and — for the cycloidal — *stability itself*: with the fixed controller rule the
two-inertia loop goes unstable below `Ω_n ≈ 68` (a Routh condition, derived live in the
cycloidal notebook). Each mechanism pays for stiffness differently: cup wall thickness
(harmonic), pin radius to the fourth power (cycloidal), cable EA and radii (capstan).

### 3 · ζ_d — structural damping ratio

**What it is.** How strongly the drive's own internal damping quenches the two-inertia
oscillation mode: `ζ_d = C/(2√(K·J_ref))`, normalized to critical for that mode.

**Where the value comes from.** Honestly: it is the one *parameter* in the table, not a
derived quantity — 0.02 for all three drives, a class-typical figure. What is derived is
the convention: `C = 2ζ_d√(K·J_ref)` with a consistent reference inertia, so the ζ you ask
for is the ζ you report (getting this inconsistent silently hands one drive extra damping
in every transient test). Note that the load-proportional friction of §η does *not* do
this job: Coulomb-type loss is a poor damper of fast modes, which is exactly why ζ_d alone
sets the cycloidal's stability floor.

**Where it bites.** The cycloidal's Routh bound (`Ω_n ≈ 68` at ζ_d = 0.02); T4's ring-down;
T5's resonance peak height.

### 4 · η_f → η_b — forward and backward efficiency, one knob

**What it is.** η_f is efficiency driving the load; η_b is efficiency when the load drives
the motor — backdrivability. The bench's structural rule: dissipation proportional to
transmitted torque makes `η_b = 2 − 1/η_f`, so the two are locked together and η_f = 0.5 is
*exactly* self-locking. Backdrivability is an output of the model; you cannot tune it to
the answer you want.

**Where the value comes from.** The loss coefficient `c = 1/η_f − 1` is derived from each
mechanism's sliding and rolling:

- **harmonic** — wave-generator bearing under mesh load
  (`μ_wg·r_b·N/r_p·(1+tanα)`) + teeth sliding through the preload each revolution
  (`N·μ_t·4w₀/(2π·r_p·tanα)`) → c = 0.29 → **η_f = 0.78, η_b = 0.71**. Plus a large
  *load-independent* no-load drag (~0.94 N·m) from the preloaded WG — the "preload tax".
- **cycloidal** — disc rolling over the pins (`μ_roll·(4/π)(1+r_pin/r_p)`) + eccentric
  bearing (`μ_b·(4/π)·r_b·N/r_p`) + output pins sliding through the eccentricity per
  output revolution (`μ_out·e/R_out`) → c = 0.045 → **η_f = 0.957, η_b = 0.955**, with
  negligible no-load drag (0.06 N·m).
- **capstan** (reference) — mostly *preload-driven*, load-independent bending hysteresis
  (`N·μ_bend·d·T_p`) → η climbs steeply with load.

The **shape** matters as much as the value: preload-dominated drives (capstan, then
harmonic) are inefficient at light load and recover as load rises; the cycloidal's curve
is flat and high from the start.

**Where it bites.** T3's breakaway torque (harmonic 2.05 mgL open, 5.89 shorted;
cycloidal 0.48/1.22); T2's energy lost per cycle (harmonic 3.23 mgL vs cycloidal 0.26).

### 5 · τ̂_max = τ_max / mgL — torque margin

**What it is.** The drive's rating in units of the arm's worst-case gravity load. 1.0
means "can just hold the arm horizontal"; below 1.0 the bench flags the config
infeasible. Above ~2, the margin above the load is deliberate steel — and *why* it is
there differs per drive, which is itself a finding.

**Where the value comes from.**

- **harmonic** — cup shear at the cone junction: `τ_max = 2π·r_cup²·t·τ_allow` with
  `τ_allow = 0.577·σ_allow/sf` (von Mises; σ_allow is a fatigue surrogate, sf = 3).
  Sized for 2× the load → 15.6 N·m → τ̂_max = 9.2.
- **cycloidal** — the smaller of output-pin bending
  (`πr³σ/(4L·sf)·n·R_out`) and pin-ring Hertz line contact
  (`2πLR*(σ/sf)²/E*` → `4τ/(z_p·r_p)`). Sized for 2× the load *plus* the control floor
  → 13.6 N·m → τ̂_max = 8.1. The margin between 2 and 8.1 is stiffness-for-control,
  not strength — the headline cycloidal finding.
- **capstan** (reference) — the smaller of Euler-Eytelwein slip
  (`2T_p·r·tanh(μβ/2)`) and cable strength → 2.0.

**Where it bites.** T4's `peak_over_rating` — the geared drives transmit 43× and 12×
their ratings on impact while the capstan is physically capped at 1.0× (it slips); and
`cfg.feasibility()`'s hold-the-arm check.

### 6 · e(·) — kinematic error, a function with a signature

**What it is.** The difference between commanded and delivered angle,
`θ_out = θ_m/N + e(θ_m)`. It is a *function*, not a scalar — each technology has a
signature frequency content — and the table quotes its amplitude in arcmin.

**Where the value comes from.** An ideal harmonic and an ideal cycloidal both have
`e = 0`: their ratios are counting arguments over conjugate profiles. All measured
ripple therefore **enters through manufacture**, and each drive's model derives it from
one or two tolerances:

- **harmonic** — cam runout Δe displaces the two engagement zones; first-order angular
  error is the runout over the pitch radius: `e = (Δe/r_p)·sin(2N·θ_m)` — 2 cycles per
  input revolution. 5 μm runout → 0.78′.
- **cycloidal** — eccentricity error repeating once per input revolution
  (`Δe_ecc/r_p`) *plus* pin-pitch error at the pin-passing frequency
  (`Δp/(2r_p)·sin(z_p·θ_out)`). Together 1.15′.
- **capstan** — ~0: no periodic mesh exists.

**Why the function matters, not just the amplitude.** The *derivative* `de/dθ` enters
the velocity map — the `(1 + de/dθ)` torque transformation at the motor port — so ripple
also ripples velocity (harmonic: ±4.6% of output speed). It shows up in T1/T2 as
position ripple at the signature frequency, and it is the term that broke energy
conservation until the integrator carried it correctly.

---

## 3. The insights, per mechanism

### Harmonic — precision bought with inertia and drag

**The ratio is a counting argument.** `N = z_f/(z_c − z_f)` = 200/2 = 100:1, exact, from
two tooth counts. Nothing about the harmonic's kinematics is a friction argument — which
is why it has zero backlash: the wave generator preloads every tooth into contact, so T1
measures ~0 lost motion (presliding only), against the cycloidal's 4.2′.

**Stiffness and strength scale together.** Both go with the cup wall thickness (Bredt:
`K = 2πGr³t/L` for the cup, plus the cone). There is *no design conflict* to trade against —
that is exactly why the package can be so compact. The price is paid elsewhere:

- **The preload tax.** The WG must deflect the rim by w₀ forever, loaded or not. Derived
  preload ≈ 206 N for this size, which becomes ~0.94 N·m of output-referred no-load drag,
  all the time. T2's energy-lost-per-cycle lands at 3.23× mgL — the largest of the three —
  and the efficiency curve starts low and climbs (0.53 at 10% load → 0.74 at rating).
- **`N²J_m`.** Π_J = 24.5: the motor's inertia, referred, is 24× the arm's own. On impact
  (T4) the motor supplies 96% of the arriving momentum and the peak transmitted torque is
  397× mgL — 43× the drive's own rating, with nothing that can slip.
- **The bandwidth window collapses.** With Π_J ≫ 1, resonance and antiresonance nearly
  coincide (ratio 1.02) — the usable control window closes. This separation is
  `√(1 + 1/Π_J)`, i.e. set by reflected inertia alone; it is the one harmonic result immune
  to the stiffness approximation below.
- **Backdrivability is predicted, and it is mediocre but real.** Derived loss (WG bearing
  under mesh load + tooth sliding under preload) gives η_f = 0.78, hence η_b = 0.714 — no
  second knob. But at 100:1 the *motor* is the real barrier: shorted, `kt²/R·N² ≈ 233
  N·m/(rad/s)` — a brake two orders of magnitude stronger than the drive's own friction.
  A high-ratio drive backdrives only insofar as its motor lets it.

**The ripple is a tolerance, made visible.** An ideal harmonic has zero transmission error
(again: counting). The measured 2-per-input-revolution ripple enters through manufacture —
cam runout over the pitch radius, 5 μm → 0.78′. Tighten the cam; the ripple shrinks
proportionally. There is no other lever.

**Honesty note.** The derived stiffness (15 kN·m/rad sized-for-the-arm; 42 kN·m/rad for a
size-typical cup) lands 2.7–5× *above* class-typical published figures because radial rim
flexure has no closed form and is not modelled. Reported, not hidden; it does not touch the
Π_J, ratio or T5-separation conclusions.

### Cycloidal — the middle ground, sized by the control loop

**The ratio is also a counting argument.** `N = z_p − 1`: a 30-lobe disc in 31 pins.
And the pin loading is exact too: `F_max = 4τ/(z_p·r_p)`.

**The headline finding: the cycloidal is sized by control, not strength.** A strength-only
sizing (2× gravity torque) gives toothpick pins and `Ω_n = 20` — and the bench's fixed
controller makes the two-inertia loop **linearly unstable** there (a Routh condition on the
closed loop; the internal mode is pumped at its own frequency). The stability floor for
this motor/arm combination is `Ω_n ≈ 68`. The reference drive is therefore built to
`Ω_n = 70`, and the resulting strength margin (8× gravity) is steel bought purely to close
the loop. This is why real cycloidal ratings always look oversized next to their joint
loads — and it is derived, not asserted, in the notebook.

**Backlash is a tolerance, and it is the distinguishing cost.** The output pins run in
holes with clearance `c`; before they engage the disc can rotate `c/R_out` each way:
`backlash = 2c/R_out` = 4.2′ for a 10 μm fit. T1 measures 4.2′ — the derived deadband,
recovered by simulation before it was simulated. Preloaded (tapered) pins delete it; it is
a choice, not a property.

**Efficiency is flat and high — the cycloidal is the most backdrivable geared drive here.**
Loss = disc rolling over the pins + eccentric bearing + pins sliding through the
eccentricity per output revolution. All three are load-proportional: η_f = 0.957, η_b =
0.955, and T3 breaks away at 0.48 mgL open-circuit (the harmonic: 2.05). The old datasheet
container's assumed 0.85 was pessimistic by a wide margin.

**The eccentric orbit costs almost nothing — inertially.** `N²·m_d·e²` is ~0.1% of the
reflected motor inertia, so Π_J = 2.26 is nearly all motor. The vibration reputation of
cycloidals is a bearing-life story (out of scope here), not an inertia story.

**Nothing breaks geometrically — the drive just gets heavy.** Sweeping the ring radius:
stiffness ∝ r², strength ∝ r, mass ∝ r², losses ratio-invariant. No bend-ratio cliff like
the capstan's, no knife-edge constraint at all — the cycloidal's binding constraints are
the control floor and mass.

**Impact: no fuse, but the middle position holds.** 98× mgL peak (12× rating) with 69% of
the momentum from the motor — a third of the harmonic's peak, nothing slips, registration
is kept, and something real would break first (no failure model — see below).

---

## 4. Where each wins

| choose the... | when you need... | and accept... |
|---|---|---|
| **harmonic** | enormous ratio in a tiny package, zero backlash, 2/rev ripple ~1′ | Π_J ≈ 24 (impact amplification, collapsed bandwidth window), η_b ≈ 0.71, a permanent preload tax, and a motor that is the real backdrive barrier |
| **cycloidal** | a *geared* drive that still backdrives (η_b ≈ 0.96), flat efficiency, moderate Π_J | genuine backlash set by your own fit tolerance, ~4× stiffness margin you must build for control, and mass |
| (capstan, reference) | backdrivability, impact tolerance, zero backlash | volume, limited stroke, ~8:1 single-stage ceiling, registration loss on slip |

**Two one-liners that survive the modelling caveats:**

- The harmonic's cost is **inertia and drag** (both derived from the ratio and the
  preload); the cycloidal's cost is **backlash and control stiffness** (both derived from
  the output pins).
- If the joint will be struck or backdriven: cycloidal over harmonic, capstan over both.
  If it will be held precisely and never touched: harmonic.

---

## 5. Caveats that qualify every number above

1. **No failure model.** T4 reports 43× and 12× rating without complaint; in reality
   something breaks first. Read the impact rows as *relative*, not absolute.
2. **Fatigue surrogates.** Both ratings use a reduced allowable stress + safety factor in
   place of a fatigue limit. Real flexsplines and pins die of crack growth.
3. **Harmonic stiffness is an upper bound** (rim flexure omitted; 2.7–5× above published).
   Everything resting on `K` specifically — absolute frequencies, not separations —
   inherits this.
4. **Cycloidal disc is rigid; contact is cylinder-on-flat.** The pins dominate the
   stiffness, so this matters little.
5. **All on one arm, one motor.** The comparison is a *configuration* comparison; Π_J and
   the ratios transfer, the specific N·m numbers do not.
