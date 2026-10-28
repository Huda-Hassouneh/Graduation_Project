"""Power and energy accounting."""

import numpy as np
import pytest

from graduation_project.config import AircraftParams
from graduation_project.energy.power import (
    EnergyIntegrator,
    electrical_power,
    ideal_induced_power,
    shaft_power,
)


def test_constant_power_integrates_exactly() -> None:
    integ = EnergyIntegrator()
    for t in np.linspace(0.0, 10.0, 101):
        integ.update(float(t), 250.0)
    assert integ.energy == pytest.approx(2500.0, rel=1e-12)  # [J]


def test_linear_power_ramp_trapezoid_is_exact() -> None:
    integ = EnergyIntegrator()
    for t in np.linspace(0.0, 2.0, 7):
        integ.update(float(t), 100.0 * t)
    assert integ.energy == pytest.approx(200.0, rel=1e-12)


def test_integrator_rejects_time_reversal() -> None:
    integ = EnergyIntegrator()
    integ.update(1.0, 1.0)
    with pytest.raises(ValueError):
        integ.update(0.5, 1.0)


def test_hover_shaft_power_matches_power_coefficient(aircraft: AircraftParams) -> None:
    """P = C_P rho n^3 D^5 per rotor at the hover speed."""
    c_p = aircraft.raw["propulsion"]["power_coefficient"]["value"]
    n = aircraft.hover_speed / (2 * np.pi)
    expected = 4 * c_p * aircraft.air_density * n**3 * aircraft.rotor_diameter**5
    p = shaft_power(np.full(4, aircraft.hover_speed), aircraft.rotor.k_m, np.ones(4))
    assert p == pytest.approx(expected, rel=1e-12)


def test_hover_figure_of_merit_is_physically_plausible(aircraft: AircraftParams) -> None:
    """Ideal induced power / shaft power must be < 1 (momentum theory bound); 0.4-0.8 is typical."""
    ideal = 4 * ideal_induced_power(aircraft.hover_thrust_per_rotor, aircraft.air_density, aircraft.rotor_diameter)
    actual = shaft_power(np.full(4, aircraft.hover_speed), aircraft.rotor.k_m, np.ones(4))
    fm = ideal / actual
    assert 0.4 < fm < 0.8


def test_electrical_power_includes_losses_and_avionics(aircraft: AircraftParams) -> None:
    shaft = 500.0
    pel = electrical_power(shaft, aircraft.power)
    assert pel == pytest.approx(shaft / aircraft.power.propulsion_efficiency + aircraft.power.avionics_power)
