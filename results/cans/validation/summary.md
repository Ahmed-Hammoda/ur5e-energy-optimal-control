# Energy validation

Electrical energy model of Clochiatti et al. (Robotica 2024), Eq. (23), with the inverse-dynamics torque of the logged motion; cycles 2 onward, mean ± std.

| run | cycles | E_elec (J) | E_mech (J) | E_cu (J) | P_peak (W) | P_avg (W) | cycle time (s) | max TCP err (mm) | placement (mm) |
|---|---|---|---|---|---|---|---|---|---|
| cans_baseline | 36 | 49.53 ± 3.95 | 17.43 ± 0.42 | 36.03 ± 4.29 | 11.50 ± 0.25 | 2.25 ± 0.18 | 22.01 ± 0.09 | 3.02 | 0.00 |
| cans_after | 24 | 35.40 ± 3.92 | 10.45 ± 1.71 | 29.72 ± 3.25 | 13.66 ± 1.56 | 1.61 ± 0.18 | 22.00 ± 0.00 | 4.00 | 0.00 |

## Energy per joint (J per cycle)

| run | base | shoulder | elbow | wrist 1 | wrist 2 | wrist 3 |
|---|---|---|---|---|---|---|
| cans_baseline | 0.12 | 23.76 | 33.14 | 5.36 | 0.00 | 0.00 |
| cans_after | 0.17 | 21.15 | 28.24 | 4.42 | 0.00 | 0.00 |

## Energy per full carton (12 cans)

| run | carton 1 | carton 2 | carton 3 |
|---|---|---|---|
| cans_baseline | 593.3 J | 593.3 J | 596.3 J |
| cans_after | 425.7 J | 424.0 J |

## Cross-check with Gazebo torque (gravity part only)

| run | E_elec, model τ (J) | E_elec, Gazebo τ (J) | Gazebo τ − model g(q), rms (N·m) |
|---|---|---|---|
| cans_baseline | 49.53 ± 3.95 | 49.45 ± 3.94 | 0.043 |
| cans_after | 35.40 ± 3.92 | 35.22 ± 3.91 | 0.051 |

## Fair-comparison checks

- **cans_baseline**: no fault: pass, cycle time <= 22.00 s: pass, placement error <= 5 mm: pass, torque within limits: pass
- **cans_after**: no fault: pass, cycle time <= 22.00 s: pass, placement error <= 5 mm: pass, torque within limits: pass
