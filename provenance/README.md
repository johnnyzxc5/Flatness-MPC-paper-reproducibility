# Source provenance

The controller/model/simulator sources and original Windows kernels were recovered byte for byte from the final paper campaign source snapshot. Their identities are recorded in source_manifest.json. The original campaign source identifier was 67629b312b5d7f79db3f3b938957f2364eb1fbefd1fc6139f5b7c59b977be04e.

Recorded result traces are excluded from this repository. Only small aggregate reference tables and three reference figures are retained. The two historical trajectory MAT dependencies are replaced by explicit endpoint configuration and a local MATLAB input generator. This changes binary input/campaign hashes while preserving the endpoints actually consumed by the simulation.

The analysis formulas originate in the verified result-audit scripts. The portable analysis adapter reads newly generated MAT outputs, checks their per-file hashes, and creates temporary intermediate arrays only under ignored results/. Formulas, case pairing, bootstrap seeds and draw order are preserved. Reproducibility requires no other repository, reviewer ledger, manuscript, AI account or external recorded-result download.

Validation receipts document independent numerical checks, a smoke test, and a complete 232-run regeneration from a fresh GitHub clone, followed by analysis and figure generation. Initial checks using historical results retained locally are identified separately. Neither those historical results nor the newly generated runs are distributed. New simulation timing measurements depend on the execution environment. See docs/VALIDATION.md for the tested scope.
