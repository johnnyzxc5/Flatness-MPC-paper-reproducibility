# Flatness-MPC paper reproducibility

Source and configuration for **Flatness-Based Predictive Control of Robotic Manipulators under Partial Actuator Degradation**, by Johnny Garcia, Nicolas Salguero, Brahim Brahmi and Naveed Iqbal.

This independent companion repository generates the benchmark simulations, tables and three paper figures. It contains the model, controller and supervisor implementations, trajectory endpoints, fault/uncertainty schedules and random seeds. **Recorded simulation traces and raw result archives are not distributed.** Four small aggregate reference tables and three reference figures are included for comparison.

## 1. Generate simulations locally

Use MATLAB R2026a with Optimization Toolbox and Robotics System Toolbox. The original Windows x64 MEX kernels are provided; on another platform, use MATLAB Coder and a configured compiler to build them first. Simulink and Parallel Computing Toolbox are not required.

```matlab
run_reproduction('build')     % optional on the recorded Windows host; required when rebuilding
run_reproduction('validate')  % independent dynamics/QP/observer checks
run_reproduction('smoke')     % 8 full-length trial runs
run_reproduction('campaign')  % generate/resume all 232 runs
```

The wrapper generates the two MATLAB trajectory input files from `config/trajectory_endpoints.json` and stages source under `.runtime/simulation_workspace/`. Each simulation is saved locally with its hash and metadata. Source and reference files remain unchanged. The completed campaign folder is under `.runtime/simulation_workspace/benchmark_evidence/results/campaign_...`; its current path is also recorded in the adjacent `latest_campaign.json`.

## 2. Generate tables and figures

Use Python 3.10 or newer:

```powershell
python -m pip install -r requirements.txt
python reproduce.py check
python reproduce.py analyze
python reproduce.py figures
```

`analyze` automatically selects the locally generated campaign, or accepts `--campaign path/to/campaign`. It checks all run hashes and pairing, recalculates control-grid and dense-grid metrics, computes the descriptive bootstrap intervals, and exports both paper tables and figure inputs. `figures` generates the three paper figures from those calculated inputs. All analysis outputs are written to ignored `results/`.

Numerical and timing results are measured anew. Wall-clock timings depend on the machine and load. Four small aggregate CSVs in `reference/` preserve the paper values as comparison targets, without requiring identical timing results on another host.

```text
simulation/frozen/    controller/model sources and original Windows MEX kernels
config/              trajectory endpoints and recorded final configuration
run_reproduction.m   isolated build/validation/smoke/full-campaign launcher
analysis/            local MAT processing, statistics, table and figure generation
reference/           small paper-level aggregate tables
figures/reference/   the three paper figures
tools/               source/configuration integrity verification
provenance/          source identities and validation receipts
```

The final campaign has 232 runs: `(54 cases + 4 integration-step repetitions) x 2 paths x 2 controllers`. Random uncertainty uses seeds 7001â€“7030 per path with paired realizations. The controller labels are `Observer` (corrected Flatness plus observer) and `LTVObserver`. The definitive executable design is `simulation/frozen/benchmark_evidence/run_evidence.m`; `config/recorded_protocol.json` records its settings. Legacy-named source folders are retained as dependencies of that frozen implementation, not as additional benchmark campaigns to execute.

See [reproduction details](docs/REPRODUCTION.md), [paper map](docs/PAPER_MAP.md), [environment](docs/ENVIRONMENT.md), [validation](docs/VALIDATION.md), [provenance](provenance/README.md), and [rights](RIGHTS.md). Project-authored materials are licensed under the [MIT License](LICENSE); third-party notices are preserved.
