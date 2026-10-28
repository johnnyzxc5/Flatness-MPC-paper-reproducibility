"""Export the two manuscript tables as portable numeric CSVs."""
from pathlib import Path
import argparse
import csv
import json

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--data', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--bundle', type=Path, required=True, help='Temporary inputs derived from the local campaign')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
args.out.mkdir(parents=True, exist_ok=True)
with (args.data / 'resumen_pareado.csv').open(encoding='utf-8-sig', newline='') as stream:
    summary = list(csv.DictReader(stream))
specifications = {
    'rmse_actual_motion_rad': ('Joint RMSE', '1e-3 rad', 1000),
    'cartesian_rmse_motion_mm': ('Cartesian position RMS', 'mm', 1),
    'TV_cmd': ('Command TV', 'N m', 1),
    'effort_cmd': ('Command quadratic effort', '(N m)^2 s', 1),
    'full_tick_p95_ms': ('Mean per-run full-update p95', 'ms', 1),
}
table = []
for row in summary:
    if row['metric'] not in specifications:
        continue
    label, unit, scale = specifications[row['metric']]
    table.append({'path': 'A' if row['profile'] == 'Inertia' else 'B', 'metric': label, 'unit': unit,
                  'Flatness_mean': float(row['flatness_mean']) * scale,
                  'LTV_mean': float(row['ltv_mean']) * scale,
                  'mean_paired_change_percent': float(row['mean_paired_change_percent']),
                  'ci95_low_percent': float(row['ci_low']), 'ci95_high_percent': float(row['ci_high']),
                  'percent_between_means': float(row['percent_between_means'])})
assert len(table) == 10
with (args.out / 'table_2_comparison.csv').open('w', encoding='utf-8', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(table[0]))
    writer.writeheader()
    writer.writerows(table)
protocol = json.loads((args.bundle / 'frozen/protocol.json').read_text(encoding='utf-8'))
meta = json.loads((args.bundle / 'portable/Inertia_healthy_Observer.json').read_text(encoding='utf-8'))
model, config, options = meta['nominal_model'], protocol['config'], protocol['options']
configuration = []
for key, unit in [('rotor', 'kg m^2'), ('fv', 'N m s/rad'), ('fc', 'N m'), ('taumax', 'N m'),
                  ('dqmax', 'rad/s'), ('ddqmax', 'rad/s^2'), ('qmax', 'rad')]:
    configuration.append({'parameter': key, 'unit': unit, 'value': json.dumps(model[key])})
for key in ['Ts', 'Np', 'Nc', 'qscale', 'effort_weight', 'smooth_multiplier', 'qp_tol', 'qp_max_iter']:
    configuration.append({'parameter': key, 'unit': 's' if key == 'Ts' else '', 'value': config[key]})
for key in ['governor_period', 'reserve']:
    configuration.append({'parameter': key, 'unit': 's' if key == 'governor_period' else '', 'value': options[key]})
for key, value, unit in [('observer_time_constant', 0.12, 's'), ('observer_torque_fraction', 0.15, ''),
                         ('health_gate', 0.015, ''), ('fallback_damping', 2, '1/s'), ('Hessian_regularization', 1e-10, '')]:
    configuration.append({'parameter': key, 'unit': unit, 'value': value})
with (args.out / 'table_1_configuration.csv').open('w', encoding='utf-8', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=['parameter', 'unit', 'value'])
    writer.writeheader()
    writer.writerows(configuration)
print('Exported table_1_configuration.csv and table_2_comparison.csv')
