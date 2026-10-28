"""Verify repository bytes and campaign input identities using only stdlib."""
from pathlib import Path
import hashlib
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


manifest = json.loads((ROOT / 'provenance/SHA256SUMS.json').read_text(encoding='utf-8'))
bad = []
for name, expected in manifest.items():
    p = ROOT / name
    if not p.is_file() or sha(p) != expected:
        bad.append(name)
sources = json.loads((ROOT / 'provenance/source_manifest.json').read_text(encoding='utf-8'))
for item in sources:
    p = ROOT / 'simulation/frozen' / item['path']
    if not p.is_file() or sha(p) != item['sha256']:
        bad.append('original source: ' + item['path'])
configuration = json.loads((ROOT / 'config/trajectory_endpoints.json').read_text(encoding='utf-8'))
assert set(configuration) == {'Inertia','Gravity'}
assert all(len(v[k]) == 6 for v in configuration.values() for k in ['q0_rad','q1_rad'])
if bad:
    print(json.dumps({'status': 'FAIL', 'mismatches': bad}, indent=2))
    raise SystemExit(1)
print(json.dumps({'status': 'PASS', 'packaged_files_verified': len(manifest),
                  'original_source_files_verified': len(sources), 'endpoint_profiles': len(configuration),
                  'numerical_recomputation': False}, indent=2))
