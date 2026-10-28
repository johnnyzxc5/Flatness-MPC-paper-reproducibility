"""Derive temporary analysis inputs from a locally generated final campaign."""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np
import pandas as pd
from scipy.io import loadmat

TRACE_KEYS = ['tc','x','qref','phase','u','health','true_health','full_tick_s','noise',
              'solver','measured','disturbance','violation','governor_s','reference_s',
              'observer_s','observer_updated','state_active','predictor_s','linearizations']
META_KEYS = ['controller','treatment','metrics','fault','config','options','nominal_model',
             'plant','observation','seed','variant','xoffset','id','static_ratio','block',
             'Tnom','ticks','preparation_s','events']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def fk(q, model):
    count = q.shape[1]
    transform = np.broadcast_to(np.eye(4), (count,4,4)).copy()
    for joint in range(6):
        rotation = np.broadcast_to(np.eye(4), (count,4,4)).copy()
        cosine, sine = np.cos(q[joint]), np.sin(q[joint])
        rotation[:,0,0] = cosine; rotation[:,0,1] = -sine
        rotation[:,1,0] = sine; rotation[:,1,1] = cosine
        transform = transform @ model['T'][:,:,joint] @ rotation
    return (transform @ model['T'][:,:,6])[:,:3,3].T


def prepare(campaign, destination):
    campaign, destination = Path(campaign).resolve(), Path(destination).resolve()
    assert (campaign / 'finished.json').is_file(), 'Finish the local 232-run campaign first.'
    source = pd.read_csv(campaign / 'metrics.csv')
    assert len(source) == 232 and source.id.nunique() == 232
    assert len(list(campaign.glob('*.mat'))) == 232
    portable, frozen = destination / 'portable', destination / 'frozen'
    portable.mkdir(parents=True, exist_ok=True); frozen.mkdir(exist_ok=True)
    enriched, index = [], []
    for row in source.to_dict(orient='records'):
        name = row['id']; file = campaign / (name + '.mat')
        sidecar = json.loads(file.with_suffix('.json').read_text(encoding='utf-8-sig'))
        digest = sha(file)
        assert digest == sidecar['sha256'], name
        result = loadmat(file, simplify_cells=True)['r']
        assert result['id'] == name
        raw, model, config = result['raw'], result['nominal_model'], result['config']
        n = int(result['ticks']); ts, h = float(config['Ts']), float(config['h'])
        assert n == 500
        x, qref, u, t = raw['x'][:,:n], raw['qref'][:,:n], raw['u'][:,:n], raw['tc'][:n]
        moving = raw['phase'][0,:n] < 1-1e-8
        error = x[:6] - qref
        position_error = fk(x[:6], model) - fk(qref, model)
        target = np.asarray(result['original_reference']['q1'])[:,None]
        position_max = np.max(abs(x[:6]-target),axis=0); velocity_max = np.max(abs(x[6:]),axis=0)
        settled = (position_max<.02) & (velocity_max<.02) & ~moving
        sustained = np.logical_and.accumulate(settled[::-1])[::-1]
        width = n * round(ts/h)
        fine_x, fine_ddq, fine_tau = raw['fine_x'][:,:width], raw['fine_ddq'][:,:width], raw['fine_tau'][:,:width]
        computed = {k:row[k] for k in ['id','profile','case_name','block','seed','controller']}
        computed.update(result['metrics'])
        computed.update(
            rmse_actual_motion_rad=float(np.sqrt(np.mean(error[:,moving]**2))),
            cartesian_rmse_motion_mm=float(1000*np.sqrt(np.mean(np.sum(position_error[:,moving]**2,axis=0)))),
            cartesian_max_mm=float(1000*np.max(np.linalg.norm(position_error,axis=0))),
            reference_motion_end_s=float(t[np.flatnonzero(moving)[-1]]+ts),
            sustained_arrival_s=float(t[np.flatnonzero(sustained)[0]]) if sustained.any() else float('nan'),
            slew_rms_cmd_Nm_s=float(np.sqrt(np.mean(np.sum((np.diff(u,axis=1)/ts)**2,axis=0)))),
            effort_applied_Nm2s=float(h*np.square(fine_tau).sum()),
            command_limit_excess_Nm=max(0,float(np.max(abs(u)-model['taumax'][:,None]))),
            saturation_fraction=float(np.mean(np.any(abs(u)>=model['taumax'][:,None]-1e-5,axis=0))),
            full_tick_p50_ms=float(np.quantile(raw['full_tick_s'][:n],.5)*1000),
            full_tick_p99_ms=float(np.quantile(raw['full_tick_s'][:n],.99)*1000),
            first_tick_ms=float(raw['full_tick_s'][0]*1000), preparation_ms=float(result['preparation_s']*1000),
            terminal_max_q_rad=float(np.max(position_max[t>=9])), terminal_max_dq_rad_s=float(np.max(velocity_max[t>=9])),
            true_static_ratio=float(result['static_ratio']))
        for key, values, limits in [('q',fine_x[:6],model['qmax']),('dq',fine_x[6:],model['dqmax']),('ddq',fine_ddq,model['ddqmax'])]:
            computed[key+'_violation_duration_s'] = float(h*np.any(abs(values)>limits[:,None]+1e-6,axis=0).sum())
        enriched.append(computed)
        arrays = {key:np.asarray(raw[key]) for key in TRACE_KEYS}
        arrays.update({'original_reference_'+key:np.asarray(result['original_reference'][key]) for key in ['q','dq','ddq','t','q0','q1']})
        arrays.update({'path_'+key:np.asarray(result['path'][key]) for key in ['s','q0','q1','d','A','C','G','floor']})
        npz = portable / (name+'.npz')
        np.savez_compressed(npz, **arrays)
        npz.with_suffix('.json').write_text(json.dumps({k:plain(result[k]) for k in META_KEYS})+'\n',encoding='utf-8')
        index.append({'id':name,'profile':row['profile'],'case_name':row['case_name'],
                      'controller':row['controller'],'raw_sha256':digest,'npz_sha256':sha(npz)})
    pd.DataFrame(enriched).to_csv(frozen/'all_metrics.csv',index=False)
    (portable/'index.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf-8')
    shutil.copy2(campaign/'protocol.json',frozen/'protocol.json')
    return destination
