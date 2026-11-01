# Mission scenarios (Phase G)

One YAML file per scenario. Loaded by `graduation_project.experiments.mission_scenarios`.

* `truth_events` and `sensor_faults` are applied to the simulated plant/sensors only. Their
  `fault_class` label is used **only by the evaluation** (detection/classification metrics); the
  research framework never sees it.
* `randomize` gives per-seed jitter: event onset `+- t_jitter` s and magnitude `x (1 +- mag_jitter)`,
  drawn from the run seed, so every seed is a different but reproducible disturbance sequence.
* Times are seconds after the start of the run (arming at 1 s, coverage starts at about 12 s and ends
  at about 100 s at nominal speed).
