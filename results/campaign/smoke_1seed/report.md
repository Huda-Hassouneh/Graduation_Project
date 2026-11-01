# Campaign `smoke_1seed` - generated summary

Runs: 52 (0 errors). Conditions: A, B, C, D. Seeds per cell: 1-1.

Values: mean [95 % bootstrap CI]. Detection/classification metrics use the scenario's fault label (evaluation only). See docs/EXPERIMENTAL_METHODOLOGY.md for definitions.

## position RMSE [m] (`pos_rmse_m`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.195 [0.195, 0.195] | 0.196 [0.196, 0.196] | 0.176 [0.176, 0.176] | 0.177 [0.177, 0.177] |
| compound | 0.124 [0.124, 0.124] | 0.091 [0.091, 0.091] | 0.121 [0.121, 0.121] | 0.088 [0.088, 0.088] |
| drag_change | 0.102 [0.102, 0.102] | 0.070 [0.070, 0.070] | 0.071 [0.071, 0.071] | 0.058 [0.058, 0.058] |
| energy_limited | 0.059 [0.059, 0.059] | 0.065 [0.065, 0.065] | 0.078 [0.078, 0.078] | 0.086 [0.086, 0.086] |
| generalization | 0.342 [0.342, 0.342] | 0.147 [0.147, 0.147] | 0.289 [0.289, 0.289] | 0.127 [0.127, 0.127] |
| invalid_measurements | 0.059 [0.059, 0.059] | 0.063 [0.063, 0.063] | 0.064 [0.064, 0.064] | 0.072 [0.072, 0.072] |
| mass_change | 0.165 [0.165, 0.165] | 0.083 [0.083, 0.083] | 0.138 [0.138, 0.138] | 0.075 [0.075, 0.075] |
| nominal | 0.059 [0.059, 0.059] | 0.065 [0.065, 0.065] | 0.059 [0.059, 0.059] | 0.065 [0.065, 0.065] |
| response_delay | 0.059 [0.059, 0.059] | 0.064 [0.064, 0.064] | 0.046 [0.046, 0.046] | 0.050 [0.050, 0.050] |
| sensor_delay | 0.316 [0.316, 0.316] | 0.281 [0.281, 0.281] | 0.095 [0.095, 0.095] | 0.098 [0.098, 0.098] |
| sensor_dropout | 0.353 [0.353, 0.353] | 0.687 [0.687, 0.687] | 0.174 [0.174, 0.174] | 0.167 [0.167, 0.167] |
| sensor_noise | 0.069 [0.069, 0.069] | 0.073 [0.073, 0.073] | 0.062 [0.062, 0.062] | 0.081 [0.081, 0.081] |
| wind_gust | 0.064 [0.064, 0.064] | 0.066 [0.066, 0.066] | 0.054 [0.054, 0.054] | 0.056 [0.056, 0.056] |

## max position error [m] (`pos_max_err_m`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 1.04 [1.04, 1.04] | 1.07 [1.07, 1.07] | 1.04 [1.04, 1.04] | 1.07 [1.07, 1.07] |
| compound | 0.52 [0.52, 0.52] | 0.28 [0.28, 0.28] | 0.52 [0.52, 0.52] | 0.23 [0.23, 0.23] |
| drag_change | 0.26 [0.26, 0.26] | 0.23 [0.23, 0.23] | 0.21 [0.21, 0.21] | 0.19 [0.19, 0.19] |
| energy_limited | 0.24 [0.24, 0.24] | 0.24 [0.24, 0.24] | 0.21 [0.21, 0.21] | 0.22 [0.22, 0.22] |
| generalization | 1.63 [1.63, 1.63] | 0.50 [0.50, 0.50] | 1.60 [1.60, 1.60] | 0.47 [0.47, 0.47] |
| invalid_measurements | 0.24 [0.24, 0.24] | 0.21 [0.21, 0.21] | 0.20 [0.20, 0.20] | 0.20 [0.20, 0.20] |
| mass_change | 0.75 [0.75, 0.75] | 0.33 [0.33, 0.33] | 0.75 [0.75, 0.75] | 0.26 [0.26, 0.26] |
| nominal | 0.24 [0.24, 0.24] | 0.24 [0.24, 0.24] | 0.24 [0.24, 0.24] | 0.24 [0.24, 0.24] |
| response_delay | 0.23 [0.23, 0.23] | 0.23 [0.23, 0.23] | 0.20 [0.20, 0.20] | 0.20 [0.20, 0.20] |
| sensor_delay | 1.15 [1.15, 1.15] | 0.81 [0.81, 0.81] | 0.20 [0.20, 0.20] | 0.23 [0.23, 0.23] |
| sensor_dropout | 1.30 [1.30, 1.30] | 2.83 [2.83, 2.83] | 0.70 [0.70, 0.70] | 0.74 [0.74, 0.74] |
| sensor_noise | 0.23 [0.23, 0.23] | 0.24 [0.24, 0.24] | 0.20 [0.20, 0.20] | 0.24 [0.24, 0.24] |
| wind_gust | 0.24 [0.24, 0.24] | 0.24 [0.24, 0.24] | 0.20 [0.20, 0.20] | 0.20 [0.20, 0.20] |

## detection rate [-] (`detected`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| compound | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| drag_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| energy_limited | n/a | n/a | n/a | n/a |
| generalization | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| invalid_measurements | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| mass_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| nominal | n/a | n/a | n/a | n/a |
| response_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_dropout | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_noise | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| wind_gust | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |

## detection delay [s] (`detection_delay_s`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | n/a | n/a | 0.3 [0.3, 0.3] | 0.3 [0.3, 0.3] |
| compound | n/a | n/a | 1.9 [1.9, 1.9] | 2.5 [2.5, 2.5] |
| drag_change | n/a | n/a | 2.9 [2.9, 2.9] | 3.1 [3.1, 3.1] |
| energy_limited | n/a | n/a | n/a | n/a |
| generalization | n/a | n/a | 1.0 [1.0, 1.0] | 1.2 [1.2, 1.2] |
| invalid_measurements | n/a | n/a | 0.5 [0.5, 0.5] | 1.1 [1.1, 1.1] |
| mass_change | n/a | n/a | 1.4 [1.4, 1.4] | 1.7 [1.7, 1.7] |
| nominal | n/a | n/a | n/a | n/a |
| response_delay | n/a | n/a | 4.9 [4.9, 4.9] | 4.2 [4.2, 4.2] |
| sensor_delay | n/a | n/a | 1.9 [1.9, 1.9] | 1.9 [1.9, 1.9] |
| sensor_dropout | n/a | n/a | 0.7 [0.7, 0.7] | 0.7 [0.7, 0.7] |
| sensor_noise | n/a | n/a | 0.2 [0.2, 0.2] | 0.1 [0.1, 0.1] |
| wind_gust | n/a | n/a | 2.0 [2.0, 2.0] | 1.8 [1.8, 1.8] |

## correct classification [-] (`classified_correct`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| compound | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| drag_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| energy_limited | n/a | n/a | n/a | n/a |
| generalization | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| invalid_measurements | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| mass_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| nominal | n/a | n/a | n/a | n/a |
| response_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_dropout | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_noise | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| wind_gust | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |

## false-alarm rate [-] (`false_alarm`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| compound | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| drag_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| energy_limited | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| generalization | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 0.00 [0.00, 0.00] |
| invalid_measurements | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| mass_change | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| nominal | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| response_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| sensor_delay | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| sensor_dropout | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| sensor_noise | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| wind_gust | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |

## recovery time [s] (`recovery_time_s`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 8.2 [8.2, 8.2] | 7.9 [7.9, 7.9] | 8.5 [8.5, 8.5] | 8.2 [8.2, 8.2] |
| compound | 4.2 [4.2, 4.2] | 41.4 [41.4, 41.4] | 3.4 [3.4, 3.4] | 0.0 [0.0, 0.0] |
| drag_change | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| energy_limited | n/a | n/a | n/a | n/a |
| generalization | 8.9 [8.9, 8.9] | 2.0 [2.0, 2.0] | 8.9 [8.9, 8.9] | 3.7 [3.7, 3.7] |
| invalid_measurements | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| mass_change | 4.9 [4.9, 4.9] | 18.1 [18.1, 18.1] | 3.4 [3.4, 3.4] | 0.0 [0.0, 0.0] |
| nominal | n/a | n/a | n/a | n/a |
| response_delay | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| sensor_delay | 13.0 [13.0, 13.0] | 23.6 [23.6, 23.6] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| sensor_dropout | 24.6 [24.6, 24.6] | 26.4 [26.4, 26.4] | 21.5 [21.5, 21.5] | 26.4 [26.4, 26.4] |
| sensor_noise | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| wind_gust | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |

## coverage [-] (`coverage_fraction`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| compound | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| drag_change | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| energy_limited | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 0.33 [0.33, 0.33] | 0.33 [0.33, 0.33] |
| generalization | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| invalid_measurements | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| mass_change | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| nominal | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| response_delay | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_delay | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_dropout | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 0.71 [0.71, 0.71] | 0.71 [0.71, 0.71] |
| sensor_noise | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| wind_gust | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |

## safe outcome [-] (`safe_outcome`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| compound | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| drag_change | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| energy_limited | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| generalization | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| invalid_measurements | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| mass_change | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| nominal | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| response_delay | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_delay | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_dropout | 1.00 [1.00, 1.00] | 0.00 [0.00, 0.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| sensor_noise | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |
| wind_gust | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] | 1.00 [1.00, 1.00] |

## min clearance [m] (`min_clearance_m`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.42 [0.42, 0.42] | 0.47 [0.47, 0.47] | 0.42 [0.42, 0.42] | 0.47 [0.47, 0.47] |
| compound | 0.92 [0.92, 0.92] | 0.92 [0.92, 0.92] | 0.92 [0.92, 0.92] | 0.92 [0.92, 0.92] |
| drag_change | 0.88 [0.88, 0.88] | 0.92 [0.92, 0.92] | 0.89 [0.89, 0.89] | 0.92 [0.92, 0.92] |
| energy_limited | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.96 [0.96, 0.96] | 0.97 [0.97, 0.97] |
| generalization | 0.85 [0.85, 0.85] | 0.93 [0.93, 0.93] | 1.08 [1.08, 1.08] | 0.93 [0.93, 0.93] |
| invalid_measurements | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| mass_change | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| nominal | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| response_delay | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| sensor_delay | 0.93 [0.93, 0.93] | 0.77 [0.77, 0.77] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| sensor_dropout | 0.65 [0.65, 0.65] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |
| sensor_noise | 0.93 [0.93, 0.93] | 0.89 [0.89, 0.89] | 0.93 [0.93, 0.93] | 0.89 [0.89, 0.89] |
| wind_gust | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] | 0.93 [0.93, 0.93] |

## energy [kJ] (`energy_total_kj`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 102 [102, 102] | 102 [102, 102] | 129 [129, 129] | 128 [128, 128] |
| compound | 115 [115, 115] | 114 [114, 114] | 159 [159, 159] | 142 [142, 142] |
| drag_change | 101 [101, 101] | 101 [101, 101] | 143 [143, 143] | 142 [142, 142] |
| energy_limited | 101 [101, 101] | 101 [101, 101] | 49 [49, 49] | 48 [48, 48] |
| generalization | 139 [139, 139] | 138 [138, 138] | 195 [195, 195] | 138 [138, 138] |
| invalid_measurements | 101 [101, 101] | 101 [101, 101] | 134 [134, 134] | 133 [133, 133] |
| mass_change | 119 [119, 119] | 119 [119, 119] | 147 [147, 147] | 119 [119, 119] |
| nominal | 101 [101, 101] | 101 [101, 101] | 101 [101, 101] | 101 [101, 101] |
| response_delay | 101 [101, 101] | 101 [101, 101] | 146 [146, 146] | 145 [145, 145] |
| sensor_delay | 102 [102, 102] | 101 [101, 101] | 153 [153, 153] | 152 [152, 152] |
| sensor_dropout | 101 [101, 101] | 101 [101, 101] | 107 [107, 107] | 106 [106, 106] |
| sensor_noise | 101 [101, 101] | 101 [101, 101] | 118 [118, 118] | 117 [117, 117] |
| wind_gust | 101 [101, 101] | 101 [101, 101] | 114 [114, 114] | 114 [114, 114] |

## diagnostic energy [kJ] (`energy_diagnostic_kj`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 6.4 [6.4, 6.4] | 6.4 [6.4, 6.4] |
| compound | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 15.4 [15.4, 15.4] | 7.7 [7.7, 7.7] |
| drag_change | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 17.9 [17.9, 17.9] | 17.9 [17.9, 17.9] |
| energy_limited | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| generalization | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 6.9 [6.9, 6.9] | 0.0 [0.0, 0.0] |
| invalid_measurements | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 25.3 [25.3, 25.3] | 25.3 [25.3, 25.3] |
| mass_change | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| nominal | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| response_delay | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 6.3 [6.3, 6.3] | 6.3 [6.3, 6.3] |
| sensor_delay | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| sensor_dropout | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| sensor_noise | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] |
| wind_gust | 0.0 [0.0, 0.0] | 0.0 [0.0, 0.0] | 5.3 [5.3, 5.3] | 5.3 [5.3, 5.3] |

## remaining energy [kJ] (`energy_remaining_kj`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 3291 [3291, 3291] | 3292 [3292, 3292] | 3264 [3264, 3264] | 3264 [3264, 3264] |
| compound | 3278 [3278, 3278] | 3278 [3278, 3278] | 3232 [3232, 3232] | 3250 [3250, 3250] |
| drag_change | 3292 [3292, 3292] | 3293 [3293, 3293] | 3250 [3250, 3250] | 3250 [3250, 3250] |
| energy_limited | 153 [153, 153] | 153 [153, 153] | 208 [208, 208] | 209 [209, 209] |
| generalization | 2831 [2831, 2831] | 2831 [2831, 2831] | 2759 [2759, 2759] | 2832 [2832, 2832] |
| invalid_measurements | 3292 [3292, 3292] | 3293 [3293, 3293] | 3259 [3259, 3259] | 3259 [3259, 3259] |
| mass_change | 3273 [3273, 3273] | 3274 [3274, 3274] | 3244 [3244, 3244] | 3274 [3274, 3274] |
| nominal | 3292 [3292, 3292] | 3293 [3293, 3293] | 3292 [3292, 3292] | 3293 [3293, 3293] |
| response_delay | 3292 [3292, 3292] | 3293 [3293, 3293] | 3246 [3246, 3246] | 3247 [3247, 3247] |
| sensor_delay | 3291 [3291, 3291] | 3293 [3293, 3293] | 3239 [3239, 3239] | 3240 [3240, 3240] |
| sensor_dropout | 3292 [3292, 3292] | 3293 [3293, 3293] | 3286 [3286, 3286] | 3287 [3287, 3287] |
| sensor_noise | 3292 [3292, 3292] | 3292 [3292, 3292] | 3275 [3275, 3275] | 3276 [3276, 3276] |
| wind_gust | 3292 [3292, 3292] | 3293 [3293, 3293] | 3279 [3279, 3279] | 3280 [3280, 3280] |

## cycle time mean [ms] (`cycle_ms_mean`)

| scenario | A | B | C | D |
|---|---|---|---|---|
| actuator_degradation | 2.11 [2.11, 2.11] | 1.98 [1.98, 1.98] | 2.65 [2.65, 2.65] | 2.79 [2.79, 2.79] |
| compound | 1.96 [1.96, 1.96] | 1.96 [1.96, 1.96] | 2.38 [2.38, 2.38] | 2.57 [2.57, 2.57] |
| drag_change | 2.09 [2.09, 2.09] | 2.22 [2.22, 2.22] | 2.75 [2.75, 2.75] | 2.82 [2.82, 2.82] |
| energy_limited | 2.09 [2.09, 2.09] | 2.25 [2.25, 2.25] | 2.73 [2.73, 2.73] | 2.72 [2.72, 2.72] |
| generalization | 2.06 [2.06, 2.06] | 2.17 [2.17, 2.17] | 2.53 [2.53, 2.53] | 2.63 [2.63, 2.63] |
| invalid_measurements | 2.03 [2.03, 2.03] | 2.13 [2.13, 2.13] | 2.45 [2.45, 2.45] | 2.53 [2.53, 2.53] |
| mass_change | 1.91 [1.91, 1.91] | 1.98 [1.98, 1.98] | 2.29 [2.29, 2.29] | 2.33 [2.33, 2.33] |
| nominal | 1.77 [1.77, 1.77] | 1.87 [1.87, 1.87] | 2.17 [2.17, 2.17] | 2.24 [2.24, 2.24] |
| response_delay | 1.79 [1.79, 1.79] | 1.88 [1.88, 1.88] | 2.26 [2.26, 2.26] | 2.35 [2.35, 2.35] |
| sensor_delay | 1.85 [1.85, 1.85] | 1.94 [1.94, 1.94] | 2.33 [2.33, 2.33] | 2.45 [2.45, 2.45] |
| sensor_dropout | 1.86 [1.86, 1.86] | 1.95 [1.95, 1.95] | 2.38 [2.38, 2.38] | 2.41 [2.41, 2.41] |
| sensor_noise | 1.94 [1.94, 1.94] | 2.17 [2.17, 2.17] | 2.43 [2.43, 2.43] | 2.70 [2.70, 2.70] |
| wind_gust | 1.97 [1.97, 1.97] | 1.93 [1.93, 1.93] | 1.95 [1.95, 1.95] | 1.91 [1.91, 1.91] |

## Paired differences (b - a, matched seeds)

| scenario | b - a | metric | mean diff [95 % CI] | Wilcoxon p | n |
|---|---|---|---|---|---|
| actuator_degradation | B - A | pos_rmse_m | 0.00177 [0.00177, 0.00177] | nan | 1 |
| actuator_degradation | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| actuator_degradation | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| actuator_degradation | B - A | energy_total_kj | -0.565 [-0.565, -0.565] | nan | 1 |
| actuator_degradation | C - A | pos_rmse_m | -0.0192 [-0.0192, -0.0192] | nan | 1 |
| actuator_degradation | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| actuator_degradation | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| actuator_degradation | C - A | energy_total_kj | 26.6 [26.6, 26.6] | nan | 1 |
| actuator_degradation | D - B | pos_rmse_m | -0.0199 [-0.0199, -0.0199] | nan | 1 |
| actuator_degradation | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| actuator_degradation | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| actuator_degradation | D - B | energy_total_kj | 26.5 [26.5, 26.5] | nan | 1 |
| actuator_degradation | D - C | pos_rmse_m | 0.001 [0.001, 0.001] | nan | 1 |
| actuator_degradation | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| actuator_degradation | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| actuator_degradation | D - C | energy_total_kj | -0.675 [-0.675, -0.675] | nan | 1 |
| compound | B - A | pos_rmse_m | -0.0333 [-0.0333, -0.0333] | nan | 1 |
| compound | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| compound | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| compound | B - A | energy_total_kj | -0.593 [-0.593, -0.593] | nan | 1 |
| compound | C - A | pos_rmse_m | -0.00303 [-0.00303, -0.00303] | nan | 1 |
| compound | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| compound | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| compound | C - A | energy_total_kj | 44.3 [44.3, 44.3] | nan | 1 |
| compound | D - B | pos_rmse_m | -0.00271 [-0.00271, -0.00271] | nan | 1 |
| compound | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| compound | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| compound | D - B | energy_total_kj | 27.6 [27.6, 27.6] | nan | 1 |
| compound | D - C | pos_rmse_m | -0.033 [-0.033, -0.033] | nan | 1 |
| compound | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| compound | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| compound | D - C | energy_total_kj | -17.3 [-17.3, -17.3] | nan | 1 |
| drag_change | B - A | pos_rmse_m | -0.032 [-0.032, -0.032] | nan | 1 |
| drag_change | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| drag_change | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| drag_change | B - A | energy_total_kj | -0.479 [-0.479, -0.479] | nan | 1 |
| drag_change | C - A | pos_rmse_m | -0.0312 [-0.0312, -0.0312] | nan | 1 |
| drag_change | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| drag_change | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| drag_change | C - A | energy_total_kj | 41.6 [41.6, 41.6] | nan | 1 |
| drag_change | D - B | pos_rmse_m | -0.0124 [-0.0124, -0.0124] | nan | 1 |
| drag_change | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| drag_change | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| drag_change | D - B | energy_total_kj | 41.5 [41.5, 41.5] | nan | 1 |
| drag_change | D - C | pos_rmse_m | -0.0132 [-0.0132, -0.0132] | nan | 1 |
| drag_change | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| drag_change | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| drag_change | D - C | energy_total_kj | -0.559 [-0.559, -0.559] | nan | 1 |
| energy_limited | B - A | pos_rmse_m | 0.00547 [0.00547, 0.00547] | nan | 1 |
| energy_limited | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| energy_limited | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| energy_limited | B - A | energy_total_kj | -0.477 [-0.477, -0.477] | nan | 1 |
| energy_limited | C - A | pos_rmse_m | 0.0187 [0.0187, 0.0187] | nan | 1 |
| energy_limited | C - A | coverage_fraction | -0.667 [-0.667, -0.667] | nan | 1 |
| energy_limited | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| energy_limited | C - A | energy_total_kj | -52.7 [-52.7, -52.7] | nan | 1 |
| energy_limited | D - B | pos_rmse_m | 0.0216 [0.0216, 0.0216] | nan | 1 |
| energy_limited | D - B | coverage_fraction | -0.667 [-0.667, -0.667] | nan | 1 |
| energy_limited | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| energy_limited | D - B | energy_total_kj | -52.8 [-52.8, -52.8] | nan | 1 |
| energy_limited | D - C | pos_rmse_m | 0.0083 [0.0083, 0.0083] | nan | 1 |
| energy_limited | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| energy_limited | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| energy_limited | D - C | energy_total_kj | -0.535 [-0.535, -0.535] | nan | 1 |
| generalization | B - A | pos_rmse_m | -0.196 [-0.196, -0.196] | nan | 1 |
| generalization | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| generalization | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| generalization | B - A | energy_total_kj | -0.218 [-0.218, -0.218] | nan | 1 |
| generalization | C - A | pos_rmse_m | -0.0537 [-0.0537, -0.0537] | nan | 1 |
| generalization | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| generalization | C - A | safe_outcome | -1 [-1, -1] | nan | 1 |
| generalization | C - A | energy_total_kj | 56.1 [56.1, 56.1] | nan | 1 |
| generalization | D - B | pos_rmse_m | -0.0198 [-0.0198, -0.0198] | nan | 1 |
| generalization | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| generalization | D - B | safe_outcome | -1 [-1, -1] | nan | 1 |
| generalization | D - B | energy_total_kj | -0.553 [-0.553, -0.553] | nan | 1 |
| generalization | D - C | pos_rmse_m | -0.162 [-0.162, -0.162] | nan | 1 |
| generalization | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| generalization | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| generalization | D - C | energy_total_kj | -56.9 [-56.9, -56.9] | nan | 1 |
| invalid_measurements | B - A | pos_rmse_m | 0.00378 [0.00378, 0.00378] | nan | 1 |
| invalid_measurements | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| invalid_measurements | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| invalid_measurements | B - A | energy_total_kj | -0.564 [-0.564, -0.564] | nan | 1 |
| invalid_measurements | C - A | pos_rmse_m | 0.00428 [0.00428, 0.00428] | nan | 1 |
| invalid_measurements | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| invalid_measurements | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| invalid_measurements | C - A | energy_total_kj | 32.7 [32.7, 32.7] | nan | 1 |
| invalid_measurements | D - B | pos_rmse_m | 0.00848 [0.00848, 0.00848] | nan | 1 |
| invalid_measurements | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| invalid_measurements | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| invalid_measurements | D - B | energy_total_kj | 32.8 [32.8, 32.8] | nan | 1 |
| invalid_measurements | D - C | pos_rmse_m | 0.00798 [0.00798, 0.00798] | nan | 1 |
| invalid_measurements | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| invalid_measurements | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| invalid_measurements | D - C | energy_total_kj | -0.46 [-0.46, -0.46] | nan | 1 |
| mass_change | B - A | pos_rmse_m | -0.0827 [-0.0827, -0.0827] | nan | 1 |
| mass_change | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| mass_change | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| mass_change | B - A | energy_total_kj | -0.596 [-0.596, -0.596] | nan | 1 |
| mass_change | C - A | pos_rmse_m | -0.0277 [-0.0277, -0.0277] | nan | 1 |
| mass_change | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| mass_change | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| mass_change | C - A | energy_total_kj | 28.1 [28.1, 28.1] | nan | 1 |
| mass_change | D - B | pos_rmse_m | -0.00747 [-0.00747, -0.00747] | nan | 1 |
| mass_change | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| mass_change | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| mass_change | D - B | energy_total_kj | 0.0305 [0.0305, 0.0305] | nan | 1 |
| mass_change | D - C | pos_rmse_m | -0.0625 [-0.0625, -0.0625] | nan | 1 |
| mass_change | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| mass_change | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| mass_change | D - C | energy_total_kj | -28.7 [-28.7, -28.7] | nan | 1 |
| nominal | B - A | pos_rmse_m | 0.00547 [0.00547, 0.00547] | nan | 1 |
| nominal | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| nominal | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| nominal | B - A | energy_total_kj | -0.477 [-0.477, -0.477] | nan | 1 |
| nominal | C - A | pos_rmse_m | 0 [0, 0] | nan | 1 |
| nominal | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| nominal | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| nominal | C - A | energy_total_kj | 0 [0, 0] | nan | 1 |
| nominal | D - B | pos_rmse_m | 0 [0, 0] | nan | 1 |
| nominal | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| nominal | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| nominal | D - B | energy_total_kj | 0 [0, 0] | nan | 1 |
| nominal | D - C | pos_rmse_m | 0.00547 [0.00547, 0.00547] | nan | 1 |
| nominal | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| nominal | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| nominal | D - C | energy_total_kj | -0.477 [-0.477, -0.477] | nan | 1 |
| response_delay | B - A | pos_rmse_m | 0.00546 [0.00546, 0.00546] | nan | 1 |
| response_delay | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| response_delay | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| response_delay | B - A | energy_total_kj | -0.367 [-0.367, -0.367] | nan | 1 |
| response_delay | C - A | pos_rmse_m | -0.0127 [-0.0127, -0.0127] | nan | 1 |
| response_delay | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| response_delay | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| response_delay | C - A | energy_total_kj | 45.1 [45.1, 45.1] | nan | 1 |
| response_delay | D - B | pos_rmse_m | -0.0138 [-0.0138, -0.0138] | nan | 1 |
| response_delay | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| response_delay | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| response_delay | D - B | energy_total_kj | 44.5 [44.5, 44.5] | nan | 1 |
| response_delay | D - C | pos_rmse_m | 0.00429 [0.00429, 0.00429] | nan | 1 |
| response_delay | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| response_delay | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| response_delay | D - C | energy_total_kj | -0.964 [-0.964, -0.964] | nan | 1 |
| sensor_delay | B - A | pos_rmse_m | -0.035 [-0.035, -0.035] | nan | 1 |
| sensor_delay | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_delay | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_delay | B - A | energy_total_kj | -1.51 [-1.51, -1.51] | nan | 1 |
| sensor_delay | C - A | pos_rmse_m | -0.22 [-0.22, -0.22] | nan | 1 |
| sensor_delay | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_delay | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_delay | C - A | energy_total_kj | 50.7 [50.7, 50.7] | nan | 1 |
| sensor_delay | D - B | pos_rmse_m | -0.183 [-0.183, -0.183] | nan | 1 |
| sensor_delay | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_delay | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_delay | D - B | energy_total_kj | 51.3 [51.3, 51.3] | nan | 1 |
| sensor_delay | D - C | pos_rmse_m | 0.00262 [0.00262, 0.00262] | nan | 1 |
| sensor_delay | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_delay | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_delay | D - C | energy_total_kj | -0.912 [-0.912, -0.912] | nan | 1 |
| sensor_dropout | B - A | pos_rmse_m | 0.334 [0.334, 0.334] | nan | 1 |
| sensor_dropout | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_dropout | B - A | safe_outcome | -1 [-1, -1] | nan | 1 |
| sensor_dropout | B - A | energy_total_kj | -0.343 [-0.343, -0.343] | nan | 1 |
| sensor_dropout | C - A | pos_rmse_m | -0.179 [-0.179, -0.179] | nan | 1 |
| sensor_dropout | C - A | coverage_fraction | -0.29 [-0.29, -0.29] | nan | 1 |
| sensor_dropout | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_dropout | C - A | energy_total_kj | 5.99 [5.99, 5.99] | nan | 1 |
| sensor_dropout | D - B | pos_rmse_m | -0.521 [-0.521, -0.521] | nan | 1 |
| sensor_dropout | D - B | coverage_fraction | -0.29 [-0.29, -0.29] | nan | 1 |
| sensor_dropout | D - B | safe_outcome | 1 [1, 1] | nan | 1 |
| sensor_dropout | D - B | energy_total_kj | 5.5 [5.5, 5.5] | nan | 1 |
| sensor_dropout | D - C | pos_rmse_m | -0.00695 [-0.00695, -0.00695] | nan | 1 |
| sensor_dropout | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_dropout | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_dropout | D - C | energy_total_kj | -0.829 [-0.829, -0.829] | nan | 1 |
| sensor_noise | B - A | pos_rmse_m | 0.00399 [0.00399, 0.00399] | nan | 1 |
| sensor_noise | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_noise | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_noise | B - A | energy_total_kj | -0.0495 [-0.0495, -0.0495] | nan | 1 |
| sensor_noise | C - A | pos_rmse_m | -0.00764 [-0.00764, -0.00764] | nan | 1 |
| sensor_noise | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_noise | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_noise | C - A | energy_total_kj | 16.7 [16.7, 16.7] | nan | 1 |
| sensor_noise | D - B | pos_rmse_m | 0.00793 [0.00793, 0.00793] | nan | 1 |
| sensor_noise | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_noise | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_noise | D - B | energy_total_kj | 15.8 [15.8, 15.8] | nan | 1 |
| sensor_noise | D - C | pos_rmse_m | 0.0196 [0.0196, 0.0196] | nan | 1 |
| sensor_noise | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| sensor_noise | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| sensor_noise | D - C | energy_total_kj | -0.904 [-0.904, -0.904] | nan | 1 |
| wind_gust | B - A | pos_rmse_m | 0.00178 [0.00178, 0.00178] | nan | 1 |
| wind_gust | B - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| wind_gust | B - A | safe_outcome | 0 [0, 0] | nan | 1 |
| wind_gust | B - A | energy_total_kj | -0.477 [-0.477, -0.477] | nan | 1 |
| wind_gust | C - A | pos_rmse_m | -0.00963 [-0.00963, -0.00963] | nan | 1 |
| wind_gust | C - A | coverage_fraction | 0 [0, 0] | nan | 1 |
| wind_gust | C - A | safe_outcome | 0 [0, 0] | nan | 1 |
| wind_gust | C - A | energy_total_kj | 12.7 [12.7, 12.7] | nan | 1 |
| wind_gust | D - B | pos_rmse_m | -0.0103 [-0.0103, -0.0103] | nan | 1 |
| wind_gust | D - B | coverage_fraction | 0 [0, 0] | nan | 1 |
| wind_gust | D - B | safe_outcome | 0 [0, 0] | nan | 1 |
| wind_gust | D - B | energy_total_kj | 12.7 [12.7, 12.7] | nan | 1 |
| wind_gust | D - C | pos_rmse_m | 0.0011 [0.0011, 0.0011] | nan | 1 |
| wind_gust | D - C | coverage_fraction | 0 [0, 0] | nan | 1 |
| wind_gust | D - C | safe_outcome | 0 [0, 0] | nan | 1 |
| wind_gust | D - C | energy_total_kj | -0.429 [-0.429, -0.429] | nan | 1 |
