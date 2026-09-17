"""Representative motor archetypes.

These are *class-typical* figures, not any specific product's datasheet. See the
package docstring. Rotor inertia is the field that matters most here: it enters
the reflected inertia as N^2 J_m, so a factor of two error in J_m is a factor of
two error in every impact result.
"""

from ..motor import Motor

MOTORS = {}


def _add(m):
    MOTORS[m.name] = m
    return m


# Small gimbal-style BLDC: many turns, high kt, low speed, very low inertia.
# The natural partner for a low-ratio drive.
_add(Motor(
    name="gimbal-4008",
    inertia=2.5e-5,      # kg*m^2
    tau_cont=0.15,       # N*m
    tau_peak=0.45,
    kt=0.33,             # N*m/A
    resistance=8.0,      # ohm
    v_bus=24.0,
    damping=2.0e-5,
))

# Larger frameless BLDC, the usual choice behind a harmonic drive.
_add(Motor(
    name="bldc-frameless-60",
    inertia=1.2e-4,
    tau_cont=0.60,
    tau_peak=2.00,
    kt=0.16,
    resistance=1.1,
    v_bus=48.0,
    damping=5.0e-5,
))

# NEMA 17 hybrid stepper, for reference: high holding torque, high rotor
# inertia for its size, which is exactly what hurts on the impact test.
_add(Motor(
    name="nema17-stepper",
    inertia=8.2e-6,
    tau_cont=0.40,
    tau_peak=0.55,
    kt=0.42,
    resistance=2.8,
    v_bus=24.0,
    damping=1.0e-4,
))

# High-torque-density robot actuator motor.
_add(Motor(
    name="bldc-quasi-direct-80",
    inertia=4.5e-4,
    tau_cont=1.8,
    tau_peak=6.0,
    kt=0.12,
    resistance=0.35,
    v_bus=48.0,
    damping=1.0e-4,
))
