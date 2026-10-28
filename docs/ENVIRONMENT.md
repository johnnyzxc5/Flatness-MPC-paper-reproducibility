# Environment

Recorded simulation: MATLAB 26.1.0.3312084 (R2026a Update 4), PCWIN64, Windows 11, Intel Core i9-13900K, one computational thread. Control period 20 ms, RK4 plant integration 1 ms, selected repetitions 0.5 ms. The controller internal integration remains 1 ms. The two original MEX files are included for numerical provenance and supported-host execution; they are not portable to Linux/macOS.

MATLAB dependencies: Optimization Toolbox (quadprog), Robotics System Toolbox (model utilities and independent checks); MATLAB Coder and compiler for building MEX. The frozen build uses `codegen`. A compatible MATLAB installation/license is required. The original configured Windows compiler was MinGW64 14.2. No Simulink model or parallel toolbox is used by the final runner.

Python: 3.10+; NumPy, SciPy, pandas and Matplotlib. `requirements.txt` provides compatible ranges; `docs/VALIDATION.md` records the actually tested versions. No notebook service or AI account is required. `check` uses only the Python standard library. All simulated outputs are generated locally; there is no recorded-data download step.
