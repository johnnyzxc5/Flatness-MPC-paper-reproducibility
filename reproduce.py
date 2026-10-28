"""Analyze locally generated simulations and generate the paper tables/figures."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
sub = parser.add_subparsers(dest='command', required=True)
sub.add_parser('check', help='Verify packaged source/configuration identities')
a = sub.add_parser('analyze', help='Process a completed local 232-run campaign')
a.add_argument('--campaign', type=Path)
a.add_argument('--out', type=Path, default=ROOT / 'results/analysis')
f = sub.add_parser('figures', help='Regenerate the three paper figures')
f.add_argument('--data', type=Path, default=ROOT / 'results/analysis/DATOS_PARA_FIGURAS')
f.add_argument('--out', type=Path, default=ROOT / 'results/figures')
args = parser.parse_args()

if args.command == 'check':
    command = [sys.executable, str(ROOT / 'tools/verify_repository.py')]
elif args.command == 'analyze':
    campaign = args.campaign
    if campaign is None:
        latest = ROOT / '.runtime/simulation_workspace/benchmark_evidence/results/latest_campaign.json'
        if not latest.is_file(): parser.error('Generate the campaign in MATLAB first, or pass --campaign.')
        campaign = Path(json.loads(latest.read_text(encoding='utf-8-sig'))['folder'])
    command = [sys.executable, str(ROOT / 'analysis/recompute.py'), '--campaign', str(campaign), '--out', str(args.out)]
elif args.command == 'figures':
    data = args.data
    if not data.is_dir():
        parser.error('Run analyze first, or provide --data with calculated figure inputs.')
    command = [sys.executable, str(ROOT / 'analysis/generate_figures.py'), '--data', str(data), '--out', str(args.out)]
code = subprocess.call(command, cwd=ROOT)
if code == 0 and args.command == 'analyze':
    code = subprocess.call([sys.executable, str(ROOT / 'analysis/export_tables.py'),
                            '--data', str(args.out / 'DATOS_PARA_FIGURAS'), '--out', str(args.out / 'tables'),
                            '--bundle', str(args.out / 'input_bundle/data')], cwd=ROOT)
raise SystemExit(code)
