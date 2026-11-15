# Energy validation

Electrical energy model of Clochiatti et al. (Robotica 2024), Eq. (23), with the inverse-dynamics torque of the logged motion; cycles 2 onward, mean ± std.

| run | cycles | E_elec (J) | E_mech (J) | E_cu (J) | P_peak (W) | P_avg (W) | cycle time (s) | max TCP err (mm) | placement (mm) |
|---|---|---|---|---|---|---|---|---|---|
| nominal_energyopt2 | 29 | 24.88 ± 1.49 | 5.82 ± 0.38 | 21.10 ± 1.20 | 14.78 ± 0.57 | 1.55 ± 0.09 | 16.02 ± 0.00 | 4.41 | 0.02 |
| nominal_scaled | 29 | 35.36 ± 0.01 | 11.96 ± 0.01 | 25.83 ± 0.00 | 12.20 ± 0.21 | 2.21 ± 0.00 | 16.02 ± 0.00 | 4.40 | 0.00 |
| nominal_trajectory | 29 | 35.34 ± 0.01 | 11.96 ± 0.01 | 25.81 ± 0.00 | 12.26 ± 0.33 | 2.21 ± 0.00 | 16.02 ± 0.00 | 3.22 | 0.00 |
| varied_scaled | 29 | 29.17 ± 0.02 | 11.78 ± 0.03 | 18.96 ± 0.01 | 15.28 ± 0.62 | 2.38 ± 0.00 | 12.28 ± 0.00 | 4.55 | 0.00 |
| varied_trajectory | 29 | 29.19 ± 0.01 | 11.78 ± 0.03 | 18.97 ± 0.00 | 15.21 ± 0.38 | 2.38 ± 0.00 | 12.28 ± 0.00 | 7.58 | 0.00 |

## Energy per joint (J per cycle)

| run | base | shoulder | elbow | wrist 1 | wrist 2 | wrist 3 |
|---|---|---|---|---|---|---|
| nominal_energyopt2 | 0.29 | 18.55 | 17.75 | 3.11 | 0.00 | 0.00 |
| nominal_scaled | 0.14 | 19.62 | 20.03 | 4.12 | 0.00 | 0.00 |
| nominal_trajectory | 0.13 | 19.61 | 20.02 | 4.12 | 0.00 | 0.00 |
| varied_scaled | 0.25 | 17.43 | 17.83 | 3.36 | 0.00 | 0.00 |
| varied_trajectory | 0.24 | 17.44 | 17.83 | 3.36 | 0.00 | 0.00 |

## Cross-check with Gazebo torque (gravity part only)

| run | E_elec, model τ (J) | E_elec, Gazebo τ (J) | Gazebo τ − model g(q), rms (N·m) |
|---|---|---|---|
| nominal_energyopt2 | 24.88 ± 1.49 | 24.61 ± 1.56 | 0.069 |
| nominal_scaled | 35.36 ± 0.01 | 35.31 ± 0.00 | 0.051 |
| nominal_trajectory | 35.34 ± 0.01 | 35.30 ± 0.00 | 0.051 |
| varied_scaled | 29.17 ± 0.02 | 29.14 ± 0.01 | 0.048 |
| varied_trajectory | 29.19 ± 0.01 | 29.15 ± 0.01 | 0.048 |

## Fair-comparison checks

- **nominal_energyopt2**: no fault: pass, cycle time <= 16.02 s: pass, placement error <= 5 mm: pass, torque within limits: pass
- **nominal_scaled**: no fault: pass, cycle time <= 16.02 s: pass, placement error <= 5 mm: pass, torque within limits: pass
- **nominal_trajectory**: no fault: pass, cycle time <= 16.02 s: pass, placement error <= 5 mm: pass, torque within limits: pass
- **varied_scaled**: no fault: pass, cycle time <= 16.02 s: pass, placement error <= 5 mm: pass, torque within limits: pass
- **varied_trajectory**: no fault: pass, cycle time <= 16.02 s: pass, placement error <= 5 mm: pass, torque within limits: pass
