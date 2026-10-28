# Paper to generator map

| Paper component | Generator or configuration |
|---|---|
| Dynamics and ZOH | simulation/frozen/benchmark/ftmpc_dynamics.m, +ftmpc/model.m, +ftmpc/linearize.m |
| Flatness/LTV predictors and QP | simulation/frozen/benchmark_v6/+ftv6/step.m |
| Disturbance observer | simulation/frozen/benchmark_v6/+ftv6/update_observer.m |
| Timing supervisor | simulation/frozen/benchmark_v5/+ftv5/govern.m and check_plan.m |
| Trajectories | config/trajectory_endpoints.json, wrapper-generated MATLAB inputs, +ftmpc/reference.m |
| 232-run design and seeds | simulation/frozen/benchmark_evidence/run_evidence.m |
| Recorded settings | config/recorded_protocol.json |
| Table I | analysis/export_tables.py, model and protocol of generated campaign |
| Table II | analysis/recompute.py, paired summary, analysis/export_tables.py |
| Figures 1–3 | analysis/generate_figures.py using the locally calculated trace/window CSVs |
| Sensitivity, initial/health errors, numerical refinement | analysis/recompute.py applied to all generated runs |

Path A is Inertia (4.5 s); Path B is GravityReverse (3 s). These historical names do not establish dynamic dominance. Observer means the final corrected Flatness with observer; LTVObserver means the LTV comparator. No alternate experimental campaign is required.
