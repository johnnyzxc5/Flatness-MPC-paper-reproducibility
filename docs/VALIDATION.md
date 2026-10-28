# Validation on 28 October 2026

This repository distributes code and configuration, not recorded run traces.

- MATLAB generated the two local trajectory inputs from JSON endpoint coordinates in a fresh runtime workspace.
- Independent model, kinematic and lifting checks passed. Mass/bias/FK differences against Robotics System Toolbox were below 4e-15; the finite-difference potential-gradient check was below 7.1e-9.
- Controller/observer/instrumentation validation passed, including an independent QP solve, observer sign/gate/bound, simulator equivalence and causal health delay.
- Eight new full-length smoke runs (2 paths x healthy/early step x 2 controllers) completed 500 updates each and met terminal criteria. Acceleration violations during step faults remain recorded outcomes; passing the terminal check is not a claim of full constraint compliance.
- The analysis pipeline processed all 232 locally retained original MAT files, checked their hashes, pairing and dense-grid metrics, and completed 5,336 numerical checks. It regenerated the paired tables, window statistics, TV decomposition and block outcomes matching all four small aggregate reference CSVs (rtol 2e-10, atol 1e-10).
- Both paper tables and the three figures were generated from the calculated outputs. No recorded trace file was required from GitHub or packaged in this repository.

The full 232-run campaign was not re-simulated during assembly. Its unchanged generator is supplied and its eight-run smoke test was executed. Full-campaign analysis was tested on the original outputs retained locally in the author's working project. Those files and the temporary analysis intermediates are not distributed here. New host timing results will vary.

Machine-readable receipts are under provenance/validation/. Tested Python: 3.12.13, NumPy 2.5.2, SciPy 1.18.1, pandas 2.3.3, Matplotlib 3.11.1. Tested MATLAB: R2026a Update 4, PCWIN64, supplied original Windows MEX kernels. Rebuilding on another platform was not tested.

Campaign hashes differ from the original paper archive because the trajectory MAT inputs are now generated from the same endpoint vectors, rather than copying the full historical trajectory files. Numerical endpoint values and controller/source formulas are unchanged.
