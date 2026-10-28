# Validation on 28 October 2026

This repository distributes code and configuration, not recorded run traces. The full generation workflow was tested from a fresh GitHub clone of source commit `a18325707c08a4a65ccd338cebb2906193c1f1ac` on the environment below.

- MATLAB generated both trajectory inputs from the JSON endpoint coordinates. No historical MAT results or trajectory files were copied into the clone.
- Independent model, kinematic and lifting checks passed. Mass/bias/FK differences against Robotics System Toolbox were below 4e-15; the finite-difference potential-gradient check was below 7.1e-9.
- Controller/observer/instrumentation validation passed, including an independent QP solve, observer sign/gate/bound, simulator equivalence and causal health delay.
- Eight full-length smoke runs (2 paths x healthy/early step x 2 controllers) completed 500 updates each and met terminal criteria. Acceleration violations during step faults remain recorded outcomes; passing the terminal check does not imply full constraint compliance.
- The fresh clone generated all 232 full-campaign simulations. The analysis checked their hashes, pairing and dense-grid metrics, completing 5,336 numerical comparisons across 116 pairs.
- Regenerated non-timing quantities matched all four small aggregate reference CSVs with zero absolute numerical difference in the compared fields. Timing and deadline exceedances were remeasured and excluded from equality checks between executions.
- Both paper tables and all three figures were generated from the new simulations. The three PDFs rendered identically to the paper references at 180 dpi in RGB.
- The clone's Git working tree remained clean after generation and analysis. Outputs were confined to ignored runtime and results directories. No recorded results were downloaded or added to Git.

Machine-readable receipts are under `provenance/validation/`. The `fresh_clone_*.json` receipts document the complete regeneration; the earlier receipts document the model checks, smoke test and initial analysis of historical results retained locally by the author.

Tested Python: 3.12.13, NumPy 2.5.2, SciPy 1.18.1, pandas 2.3.3, Matplotlib 3.11.1. Tested MATLAB: R2026a Update 4, PCWIN64, supplied original Windows MEX kernels. Rebuilding on another platform was not tested. Wall-clock timings depend on machine and load.

Campaign hashes differ from the original paper archive because the trajectory MAT inputs are generated from the same endpoint vectors, rather than copying the full historical trajectory files. Numerical endpoint values and controller/source formulas are unchanged.
