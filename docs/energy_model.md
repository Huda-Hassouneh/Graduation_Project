# Energy model

Equations: [model_equations.md §5](model_equations.md). Code: `energy/power.py`.

* **Shaft power** `P_shaft = Σ η_i k_m Ω_i³`, consistent with the `C_P` propeller model.
* **Electrical power** `P_el = P_shaft / η_prop + P_avionics`. Constant efficiency is an
  approximation. Real motor and ESC efficiency varies with load and speed.
* **Energy** `E = ∫ P_el dt`, trapezoidal at the plant step. Reported per run as `energy_J`,
  `mean_power_W` and `energy_per_m_J` (energy / path length).

**Status:** approximate. The model is labelled *not validated* for the real Matrice 400.
Its hover power implies a 47 min hover against DJI's published 53 min (aircraft_reference.md
§4), and that is a consistency check only. Not yet modelled: battery voltage sag and internal
resistance, temperature, motor acceleration power (`J_rotor Ω Ω̇`), and regeneration.
Planned (Milestone 6): remaining-energy estimate with uncertainty, energy attributed to
diagnostic and recovery actions, and sensitivity to `C_P`, `η_prop` and `P_avionics`.
