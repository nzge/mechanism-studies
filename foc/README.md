# FOC — field-oriented control

Kept as its own directory rather than a single notebook, because it is a different
problem from the rest of this repo.

`drivelab` studies **transmissions**: what happens between a motor shaft and a load.
It treats the motor as an ideal torque source with three properties that matter —
rotor inertia, a torque ceiling, and `kt^2/R` damping when the phases are shorted.
That abstraction is deliberate and it is enough for every question the bench asks.

FOC lives on the other side of that boundary: current loops, commutation, the dq
transform, sensor and observer behaviour, PWM and dead-time. It needs its own models,
its own time scales (tens of kHz against the bench's tens of Hz), and its own test
rigs — hence a directory, not a notebook.

## `foc-simulator.html` — interactive FOC lab

A self-contained, dependency-free single file (open it in any browser, or serve it:
`python3 -m http.server` from the repo root and visit
`http://127.0.0.1:8123/foc/foc-simulator.html`).

**Model.** Surface PMSM (Rs, Ld, Lq, λm, pole pairs) on a control arm with gravity
load `m·g·Lc·sin θm`, cogging `Acog·sin 6θe`, Coulomb + viscous friction and an
injectable external disturbance. Electrical + mechanical states integrated at 2 µs
(RK4); the inverter is an SVPWM average model with sample-and-hold at fPWM
(2–20 kHz), midpoint injection and the `|v| ≤ Vdc/√3` voltage circle.

**Control chain** (all of it live on screen): abc sense → Clarke → Park → dq PI with
decoupling feed-forward and back-calculation anti-windup → Park⁻¹ → SVPWM → VSI.
Cascades: current → speed (PI) → position (P on error + D on measured speed). Gains
are sliders; the position D term is not cosmetic — see the inverted-pendulum note
under "verified".

**Classic motor tests, as modes:**

| mode | what it does |
|---|---|
| idle / open-circuit | inverter off, arm free-falls; back-EMF visible on phases |
| V/Hz open-loop | soft ramp, IR boost; shows pull-in oscillation and loss of sync at low V |
| current (torque) loop | torque step, `Te/iq = 1.5·pp·λm` constant check, locked-rotor option |
| speed loop | step + regulation; load-pulse injection shows droop and recovery |
| position loop | lift and hold against gravity; at rest `Te ≡ T_gravity` |
| back-EMF (scripted) | spin up → hold → open terminals → measure Vll, λm, Ke |
| resistance (scripted) | locked-rotor DC step → Rs = V/I, L = Rs·τ₆₃.₂ |

**Telemetry.** Control arm animation (stator ring coloured by phase currents, rotor
pole pattern at θe, control/non-control torque arcs, gravity arrow); phasor diagram
(abc phasors, their literal sum, rotating dq frame with id/iq decomposition, θe);
scopes for i_abc, v_abc, i_dq, torque (control vs non-control vs gravity), and
θ/ω with references; a live numeric signal-flow chain; a stat-chip strip.

**Verified.** The physics core is unit-tested headlessly (Node) — resistance and
back-EMF scripts recover the true Rs/L/λm to <5 %, position hold balances gravity to
10⁻⁵ N·m, `kt` is exact, V/Hz pull-in succeeds at rated V and fails below it — and
the full page is smoke-tested in headless Chrome (zero console errors, scripted tests
complete in-browser). Position hold works at any angle including the fully inverted
θ* = 180°; the top-hold is an inverted pendulum, and the linearized cascade only
satisfies the Routh condition `a·b > c` there because of the Kp θ′ damping term —
set it to 0 and watch the top-hold enter a torque-saturated limit cycle.

## The seam between them

The one place the two meet is `drivelab/motor.py`. If work here produces a better motor
model — torque ripple, current-loop bandwidth, thermal derating, saturation — it belongs
behind that same interface so the transmission bench inherits it without changes.

Worth noting what the bench already shows about that seam: in the T3 backdrive test the
short-circuit damping term raises the capstan's breakaway torque from 0.33 to 0.41 times
the gravity load. For a low-ratio drive, the *motor* is a substantial part of what
resists backdriving. Anything FOC does to the effective impedance at the shaft shows up
directly in transmission behaviour.
