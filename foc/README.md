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

## The seam between them

The one place the two meet is `drivelab/motor.py`. If work here produces a better motor
model — torque ripple, current-loop bandwidth, thermal derating, saturation — it belongs
behind that same interface so the transmission bench inherits it without changes.

Worth noting what the bench already shows about that seam: in the T3 backdrive test the
short-circuit damping term raises the capstan's breakaway torque from 0.33 to 0.41 times
the gravity load. For a low-ratio drive, the *motor* is a substantial part of what
resists backdriving. Anything FOC does to the effective impedance at the shaft shows up
directly in transmission behaviour.

## Not started yet

Nothing here but this note.
