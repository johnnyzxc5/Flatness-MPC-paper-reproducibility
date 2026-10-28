# Evidence closure protocol — fixed before running

Purpose: confirm the provisional Observer and LTVObserver implementations from v6, without retuning either controller. This is simulation evidence, not a stability or hardware certificate. Historical campaigns remain available; no adverse result is removed.

Two existing paths: Inertia (4.5 s, final effectiveness .65) and GravityReverse (3 s, .60). Names identify legacy endpoints, not dominant physical forces. Both use the identical constant-health, path-preserving governor, observer, objective, constraints, sample time .02 s, 10 s observation window and physical integration .001 s. Known current effectiveness is the main assumption. No future fault values are passed to controllers. Same seed/noise/reference per pair; method order alternates.

Per path, 54 cases × 2 methods = 108 runs:
- 30 new paired seeds 7001:7030 with the original random mass/inertia/CoM/friction ranges and state noise. Fault shape cycles step/.3 s ramp/2 s ramp, onset cycles .5 s/.45T/.7T. These new seeds confirm selection on previously used paths; they do not establish geometric generalization.
- Healthy nominal, plus 3 fault shapes × 3 onsets: .2T/.5T/.8T (10 cases).
- Mass changes −20%, −10%, +10%, +20%, +30%, and friction changes −25%, +25%, +50% (8). Mass-only perturbation is a sensitivity parameter, not an equivalent end-effector payload.
- Supplementary fault-information stress: effectiveness bias ±.03, delay 2 samples, or Gaussian effectiveness noise .01 (4). Separate from the assumed known-fault main comparison. No FDI algorithm is claimed.
- Adaptation challenge: early .3 s ramp to .555 (Inertia) / .50 (GravityReverse), matching v5 authority-limited cases (1).
- Initial position +.03 rad and velocity +.05 rad/s on all joints, healthy (1). Retain any failures of the hard-constrained approximation.

Repeat 4 cases (healthy, mid-motion step, mass +30%, adaptation) per path/method at physical integration .0005 s: 16 convergence runs. Total: 232. Internal observer/model integration remains .001 s. All true-path static authority ratios are recorded; out-of-scope cases are recorded explicitly, never replaced. Static sampling is a scope screen, not a viability proof.

Metrics: task success; adapted joint and Cartesian error during the actual reference motion; terminal error; task arrival time; command/applied effort (Nm²s); discrete command TV and RMS sample-to-sample slew; spectra; physical q/dq/ddq violations (peak and duration); saturation, fallback, governor failures; preparation, first step, full step p50/p95/p99/max, deadline exceedances. Compute costs include the observer, reference generation and governor, exclude simulated plant integration, and do not model execution delay in the plant. No timing outliers discarded. Confidence intervals: paired descriptive bootstrap, 10,000 resamples, 30 seeds per path; preserve absolute metrics and both methods.

Closure: raw MAT + hashes + frozen source; all 232 statuses accounted for; equality of paired input/noise/reference checked; convergence assessed; 49 legacy reviewer IDs mapped to evidence and outstanding manuscript/theory/literature work. Raw annotated reviewer PDF is unavailable: this is coverage of the supplied ledger, not independent verification of its transcription.
