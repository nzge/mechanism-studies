"""Named reference configurations -- the standardized subjects of the bench.

A comparison is only as good as its subjects. These exist so that "the capstan
result" means one specific, inspectable, reproducible system rather than
whatever parameters happened to be in a notebook cell.

Provenance, again: the capstan entries are sized from first principles by
`CapstanDrive.sized_for`. The harmonic and cycloidal entries are
`GenericGearedDrive` containers holding class-typical *published* figures. Both
are legitimate bench subjects, but only the first tells you anything about why
the numbers are what they are.
"""

import math

from ..config import Config
from ..drives.capstan import CapstanDrive
from ..drives.ideal import GenericGearedDrive, CompliantIdealDrive
from .motors import MOTORS
from .cables import CABLES
from .arms import ARMS


def reference_capstan(arm="desktop-300", motor="bldc-frameless-60",
                      R_output=0.090, N=8.0, cable="steel-7x19",
                      torque_margin=2.0, n_wraps=3.0, mu=0.2,
                      name=None, **kw):
    """A capstan sized to carry `torque_margin` times the arm's gravity load."""
    a = ARMS[arm] if isinstance(arm, str) else arm
    m = MOTORS[motor] if isinstance(motor, str) else motor
    c = CABLES[cable] if isinstance(cable, str) else cable
    drive = CapstanDrive.sized_for(
        tau_required=torque_margin * a.gravity_moment,
        R_output=R_output, N=N, cable=c, n_wraps=n_wraps, mu=mu,
        free_length=2.0 * R_output, capstan_width=None,
        inertia_ref=a.inertia,
        name=name or ("capstan-%.0f:1" % N), **kw)
    return Config(motor=m, drive=drive, plant=a,
                  name=name or ("capstan %.1f:1 on %s" % (drive.N, a.name)))


def harmonic_like(arm="desktop-300", motor="bldc-frameless-60", N=100.0,
                  eta_f=0.75, stiffness=7.0e3, input_drag=0.015,
                  error_arcmin=1.0, rated_torque=8.0,
                  name="harmonic-100 (datasheet)"):
    """Class-typical harmonic drive figures in a GenericGearedDrive.

    Not derived. The three numbers that drive every result are the efficiency
    (which sets backdrivability through eta_b = 2 - 1/eta_f), the no-load drag
    (large, because the wave generator is preloaded, and multiplied by N when
    referred to the output), and N^2 J_m (enormous at 100:1). The classic
    2-cycles-per-input-revolution ripple is included.
    """
    a = ARMS[arm] if isinstance(arm, str) else arm
    m = MOTORS[motor] if isinstance(motor, str) else motor
    drive = GenericGearedDrive(
        N=N, eta_forward=eta_f, stiffness=stiffness,
        no_load_drag=input_drag * N,          # output-referred
        backlash=0.0,                          # harmonic drives: ~zero
        error_amplitude=math.radians(error_arcmin / 60.0),
        error_cycles=2.0,                      # per wave-generator revolution
        inertia_extra=2.0e-5, stiction_ratio=1.8,
        tau_rated=rated_torque, structural_damping_ratio=0.02,
        name=name, inertia_ref=a.inertia)
    return Config(motor=m, drive=drive, plant=a, name=name)


def cycloidal_like(arm="desktop-300", motor="bldc-frameless-60", N=30.0,
                   eta_f=0.85, stiffness=2.0e4, input_drag=0.012,
                   error_arcmin=0.7, lobes=10.0, backlash_arcmin=1.5,
                   rated_torque=12.0, name="cycloidal-30 (datasheet)"):
    """Class-typical cycloidal reducer figures. Not derived.

    Higher efficiency and stiffness than the harmonic, at the cost of a little
    backlash, plus ripple at the lobe-passing frequency.
    """
    a = ARMS[arm] if isinstance(arm, str) else arm
    m = MOTORS[motor] if isinstance(motor, str) else motor
    drive = GenericGearedDrive(
        N=N, eta_forward=eta_f, stiffness=stiffness,
        no_load_drag=input_drag * N,
        backlash=math.radians(backlash_arcmin / 60.0),
        error_amplitude=math.radians(error_arcmin / 60.0),
        error_cycles=lobes,
        inertia_extra=3.0e-5, stiction_ratio=1.6,
        tau_rated=rated_torque, structural_damping_ratio=0.02,
        name=name, inertia_ref=a.inertia)
    return Config(motor=m, drive=drive, plant=a, name=name)


def ideal_reference(arm="desktop-300", motor="bldc-frameless-60", N=8.0,
                    stiffness=1.0e4, name="ideal (lossless)"):
    """Lossless, backlash-free, no error. The control case -- every metric that
    is a *loss* must go to zero here, and every test that does not report zero
    is measuring its own artifacts."""
    a = ARMS[arm] if isinstance(arm, str) else arm
    m = MOTORS[motor] if isinstance(motor, str) else motor
    drive = CompliantIdealDrive(
        N=N, stiffness=stiffness,
        damping=0.02 * 2.0 * (stiffness * a.inertia) ** 0.5, name=name)
    return Config(motor=m, drive=drive, plant=a, name=name)


REFERENCE_CONFIGS = {
    "capstan": reference_capstan,
    "harmonic": harmonic_like,
    "cycloidal": cycloidal_like,
    "ideal": ideal_reference,
}
