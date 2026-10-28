#!/usr/bin/env python3
"""Analyze locally generated MAT outputs and compute the paper statistics.

Usage: python analysis/recompute.py --campaign PATH --out PATH
No controller simulation is executed by this Python analysis step.
"""
from pathlib import Path
import sys, json, hashlib, platform
import numpy as np
import pandas as pd
import scipy
from scipy.io import loadmat

import argparse
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--campaign', type=Path, required=True, help='Completed local campaign folder with 232 MAT/JSON pairs')
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[1]
OUT = args.out.resolve()
assert OUT != ROOT / 'data' and (ROOT / 'data') not in OUT.parents
D = OUT / 'DATOS_PARA_FIGURAS'
D.mkdir(parents=True, exist_ok=True)
E = OUT / 'input_bundle/data'
P = E / 'portable'
RAW = args.campaign.resolve()
from prepare_inputs import prepare
prepare(RAW, E)

M = pd.read_csv(E / 'frozen/all_metrics.csv')
assert len(M) == 232 and M.id.nunique() == 232
assert M.groupby(['profile','case_name']).size().eq(2).all()
PROFILES = ['Inertia','GravityReverse']
KINDS = ['Observer','LTVObserver']
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks, windows, fine_rows, fine_check, trace_rows = [], [], [], [], []
arrays, metas = {}, {}
input_hashes = {}
def record(p):
    input_hashes[str(p.relative_to(OUT)) if p.is_relative_to(OUT) else p.name] = sha(p)
def close(id_, metric, actual, reference, rtol=2e-11, atol=1e-12):
    assert np.isclose(actual, reference, rtol=rtol, atol=atol, equal_nan=True), (id_,metric,actual,reference)
    checks.append(dict(id=id_,metric=metric,recomputed=float(actual),received=float(reference),absolute_difference=float(abs(actual-reference))))
def fk(q, model):
    n=q.shape[1]; T=np.broadcast_to(np.eye(4),(n,4,4)).copy()
    base=np.array(model['T'])
    for j in range(6):
        R=np.broadcast_to(np.eye(4),(n,4,4)).copy()
        cs=np.cos(q[j]); sn=np.sin(q[j])
        R[:,0,0]=cs; R[:,0,1]=-sn; R[:,1,0]=sn; R[:,1,1]=cs
        T=T@base[:,:,j]@R
    return (T@base[:,:,6])[:,:3,3].T
index={r['id']:r for r in json.loads((P/'index.json').read_text())}
for row in M.itertuples(index=False):
    f=P/(row.id+'.npz'); jf=f.with_suffix('.json')
    assert sha(f)==index[row.id]['npz_sha256']
    record(f);record(jf)
    with np.load(f,allow_pickle=False) as zz: z={k:zz[k] for k in zz.files}
    meta=json.loads(jf.read_text());arrays[row.id]=z;metas[row.id]=meta
    n=int(meta['ticks']); ts=meta['config']['Ts']
    assert n==500 and ts==.02
    t=z['tc'][:n];x=z['x'][:,:n];u=z['u'][:,:n];er=x[:6]-z['qref'][:,:n]
    moving=z['phase'][0,:n]<1-1e-8
    pe=fk(x[:6],meta['nominal_model'])-fk(z['qref'][:,:n],meta['nominal_model'])
    vals={
        'TV_cmd':abs(np.diff(u,axis=1)).sum(),
        'effort_cmd':ts*np.square(u).sum(),
        'rmse_actual_motion_rad':np.sqrt(np.mean(np.square(er[:,moving]))),
        'cartesian_rmse_motion_mm':1e3*np.sqrt(np.mean(np.sum(pe[:,moving]**2,axis=0))),
        'slew_rms_cmd_Nm_s':np.sqrt(np.mean(np.sum((np.diff(u,axis=1)/ts)**2,axis=0))),
        'full_tick_p95_ms':1e3*np.quantile(z['full_tick_s'],.95,method='hazen'),
        'full_tick_max_ms':1e3*z['full_tick_s'].max(),
        'first_tick_ms':1e3*z['full_tick_s'][0],
        'deadline_exceedances':np.sum(z['full_tick_s']>ts),
        'command_limit_excess_Nm':max(0,float(np.max(abs(u)-np.array(meta['nominal_model']['taumax'])[:,None]))),
        'saturation_fraction':np.mean(np.any(abs(u)>=np.array(meta['nominal_model']['taumax'])[:,None]-1e-5,axis=0)),
        'fallback_fraction':np.mean(z['solver'][:,1]),
        'terminal_max_q_rad':np.max(abs(x[:6,t>=9]-z['original_reference_q1'][:,None])),
        'terminal_max_dq_rad_s':np.max(abs(x[6:,t>=9])),
        'reference_motion_end_s':t[np.flatnonzero(moving)[-1]]+ts,
    }
    for k in ('q','dq','ddq'):
        # These NPZ entries are the peaks calculated on all integration steps,
        # including both interval ends, not accelerations differentiated at Ts.
        vals['max_'+k+'_violation']=z['violation'][('q','dq','ddq').index(k)].max()
    for k,v in vals.items():close(row.id,k,v,getattr(row,k))
    goal=int(np.all(abs(x[:6,t>=9]-z['original_reference_q1'][:,None])<.02) and np.all(abs(x[6:,t>=9])<.02))
    close(row.id,'goal_reached',goal,row.goal_reached)
    assert goal==int((not moving[t>=9].any()) and goal)
    if row.block!='convergence':
        for window,mask in [('all',np.ones(n,bool)),('motion',moving),('hold',~moving),('tail',t>=9)]:
            dm=mask[:-1]&mask[1:];du=np.diff(u,axis=1)[:,dm]
            windows.append(dict(id=row.id,profile=row.profile,case=row.case_name,block=row.block,controller=row.controller,window=window,
                samples=int(mask.sum()),increments=int(dm.sum()),duration_s=ts*mask.sum(),TV=abs(du).sum(),
                slew=np.sqrt(np.mean(np.sum((du/ts)**2,axis=0))),rmse=np.sqrt(np.mean(er[:,mask]**2)),
                velocity_rms=np.sqrt(np.mean(x[6:,mask]**2)),effort=ts*np.square(u[:,mask]).sum()))
    # All 20 early-step / limited-authority? Only 8 runs selected for main figures.
    if row.case_name in ['shape1_onset1','limited_authority']:
        for k in range(n):
            trace_rows.append(dict(id=row.id,profile=row.profile,case_name=row.case_name,controller=row.controller,
                time_s=t[k],q2_rad=x[1,k],qref2_rad=z['qref'][1,k],dq2_rad_s=x[7,k],
                joint_error_norm_mrad=1e3*np.linalg.norm(er[:,k]),u2_Nm=u[1,k],
                tau2_control_Nm=z['true_health'][1,k]*u[1,k],alpha2_true=z['true_health'][1,k],
                alpha2_known=z['health'][1,k],available_torque2_Nm=z['true_health'][1,k]*meta['nominal_model']['taumax'][1],
                phase=z['phase'][0,k],phase_speed_s_inv=z['phase'][1,k],
                phase_accel_s_inv2=z['phase'][2,k],full_tick_ms=1e3*z['full_tick_s'][k]))

for _,group in M.groupby(['profile','case_name'],sort=False):
    a,b=[arrays[group.loc[group.controller==k,'id'].item()] for k in KINDS]
    for key in ['noise','health','true_health','qref','phase']:
        assert np.array_equal(a[key],b[key]),(group.id.tolist(),key)
    ma,mb=[metas[group.loc[group.controller==k,'id'].item()] for k in KINDS]
    for key in ['plant','observation','fault','xoffset','original_reference' ]:
        if key in ma:assert ma[key]==mb[key],key

W=pd.DataFrame(windows)
W.to_csv(D/'ventanas_recalculadas.csv',index=False)
# Preserve seed/order/bootstrap convention of the received analysis exactly.
metrics=['rmse_actual_motion_rad','cartesian_rmse_motion_mm','TV_cmd','slew_rms_cmd_Nm_s','effort_cmd','effort_applied_Nm2s','full_tick_p95_ms']
rng=np.random.default_rng(20261018); paired=[];individual=[]
for profile in PROFILES:
    rr=M.loc[(M.profile==profile)&(M.block=='random')]
    draw=rng.integers(0,30,(10000,30))
    for metric in metrics:
        a=rr.loc[rr.controller==KINDS[0]].sort_values('seed')[metric].to_numpy()
        b=rr.loc[rr.controller==KINDS[1]].sort_values('seed')[metric].to_numpy()
        pc=100*(a/b-1);ci=np.quantile(pc[draw].mean(axis=1),[.025,.975])
        paired.append(dict(profile=profile,metric=metric,n=30,flatness_mean=a.mean(),ltv_mean=b.mean(),
            mean_paired_change_percent=pc.mean(),ci_low=ci[0],ci_high=ci[1],
            percent_between_means=100*(a.mean()/b.mean()-1),wins=int((a<b).sum()),ties=int((a==b).sum()),
            minimum_paired_change_percent=pc.min(),maximum_paired_change_percent=pc.max()))
        for seed,x,y,v in zip(range(7001,7031),a,b,pc):
            individual.append(dict(profile=profile,seed=seed,metric=metric,flatness=x,ltv=y,paired_change_percent=v))
S=pd.DataFrame(paired);S.to_csv(D/'resumen_pareado.csv',index=False)
pd.DataFrame(individual).to_csv(D/'efectos_60_parejas.csv',index=False)
rng=np.random.default_rng(20261019);win_summary=[]
for profile in PROFILES:
    for window in ['motion','hold','tail']:
        rr=W.loc[(W.profile==profile)&(W.block=='random')&(W.window==window)]
        for metric in ['TV','slew','rmse','velocity_rms']:
            a=rr.loc[rr.controller==KINDS[0]].sort_values('case')[metric].to_numpy()
            b=rr.loc[rr.controller==KINDS[1]].sort_values('case')[metric].to_numpy()
            pc=100*(a/b-1);ci=np.quantile(rng.choice(pc,(10000,30),replace=True).mean(axis=1),[.025,.975])
            win_summary.append(dict(profile=profile,window=window,metric=metric,flatness_mean=a.mean(),ltv_mean=b.mean(),paired_change=pc.mean(),ci_low=ci[0],ci_high=ci[1]))
WS=pd.DataFrame(win_summary);WS.to_csv(D/'resumen_ventanas.csv',index=False)
decomp=[]
for profile in PROFILES:
    rr=W.loc[(W.profile==profile)&(W.block=='random')]
    vals={}
    for kind in KINDS:
        vals[kind]={w:rr.loc[(rr.controller==kind)&(rr.window==w),'TV'].mean() for w in ['all','motion','hold']}
        vals[kind]['boundary']=vals[kind]['all']-vals[kind]['motion']-vals[kind]['hold']
    total=vals[KINDS[1]]['all']-vals[KINDS[0]]['all']
    for w in ['all','motion','hold','boundary']:
        delta=vals[KINDS[1]][w]-vals[KINDS[0]][w]
        decomp.append(dict(profile=profile,window=w,flatness_TV_mean_Nm=vals[KINDS[0]][w],ltv_TV_mean_Nm=vals[KINDS[1]][w],
            absolute_saving_mean_Nm=delta,share_of_total_absolute_saving_percent=100*delta/total))
pd.DataFrame(decomp).to_csv(D/'aporte_absoluto_TV.csv',index=False)

# Optional dense MAT verification, using all 232 downloaded original runs.
for f in (sorted(RAW.glob('*.mat')) if RAW else []):
    id_=f.stem;record(f);receipt=json.loads(f.with_suffix('.json').read_text());record(f.with_suffix('.json'))
    assert sha(f)==receipt['sha256']==index[id_]['raw_sha256']
    r=loadmat(f,simplify_cells=True)['r'];raw=r['raw'];n=int(r['ticks']);h=r['config']['h'];ns=round(r['config']['Ts']/h);row=M.loc[M.id==id_].iloc[0]
    for key in ['x','u','qref','phase','full_tick_s','noise','true_health']:
        assert np.array_equal(raw[key],arrays[id_][key]),(id_,key)
    fine=raw['fine_x'][:,:n*ns];acc=raw['fine_ddq'][:,:n*ns];tau=raw['fine_tau'][:,:n*ns]
    close(id_,'effort_applied_Nm2s_from_MAT',h*np.square(tau).sum(),row.effort_applied_Nm2s)
    for key,values,limits in [('q',fine[:6],r['nominal_model']['qmax']),('dq',fine[6:],r['nominal_model']['dqmax']),('ddq',acc,r['nominal_model']['ddqmax'])]:
        excess=abs(values)-np.asarray(limits)[:,None];mask=np.any(excess>1e-6,axis=0)
        close(id_,key+'_violation_duration_s',h*mask.sum(),row[key+'_violation_duration_s'])
        runs=np.diff(np.r_[False,mask,False].astype(int));starts=np.flatnonzero(runs==1);ends=np.flatnonzero(runs==-1)
        fine_check.append(dict(id=id_,quantity=key,unit={'q':'rad','dq':'rad/s','ddq':'rad/s^2'}[key],
            peak_excess_from_full_intervals=row['max_'+key+'_violation'],
            peak_excess_from_stored_grid=max(0,float(excess.max())),total_violation_duration_s=h*mask.sum(),
            longest_grid_episode_s=0 if not len(starts) else h*max(ends-starts),episodes=len(starts),
            first_grid_violation_s=np.nan if not len(starts) else h*starts[0]))
    if row.case_name in ['shape1_onset1','limited_authority']:
        t=raw['t'];fault=r['fault'];z=np.clip((t-fault[0])/max(np.finfo(float).eps,fault[1]-fault[0]),0,1)
        alpha=1-(1-fault[2])*z
        for k in range(len(t)):
            fine_rows.append(dict(id=id_,profile=row.profile,case_name=row.case_name,controller=row.controller,
                time_s=t[k],q2_rad=raw['fine_x'][1,k],dq2_rad_s=raw['fine_x'][7,k],ddq2_rad_s2=raw['fine_ddq'][1,k],
                tau2_Nm=raw['fine_tau'][1,k],alpha2_true=alpha[k],available_torque2_Nm=alpha[k]*r['nominal_model']['taumax'][1],
                acceleration_limit2_rad_s2=r['nominal_model']['ddqmax'][1],
                acceleration_utilization_all_joints=np.max(abs(raw['fine_ddq'][:,k])/r['nominal_model']['ddqmax']),
                worst_acceleration_joint_one_based=1+int(np.argmax(abs(raw['fine_ddq'][:,k])/r['nominal_model']['ddqmax']))))
pd.DataFrame(fine_check).to_csv(D/'comprobacion_MAT.csv',index=False)
pd.DataFrame(fine_rows).to_csv(D/'trazas_finas_figuras.csv',index=False)
if RAW is None:
    import shutil
    shutil.copy2(E/'reference/trazas_finas_figuras.csv', D/'trazas_finas_figuras.csv')
else:
    assert len(list(RAW.glob('*.mat'))) == 232, 'Expected all 232 original MAT files'

pd.DataFrame(trace_rows).to_csv(D/'trazas_control_figuras.csv',index=False)

# Detailed deterministic families retain all cases, never a selected winner.
det=M.loc[~M.block.isin(['random','convergence'])].copy()
det.to_csv(D/'familias_deterministas_completas.csv',index=False)
detpaired=[]
for (profile,case),group in det.groupby(['profile','case_name'],sort=False):
    a,b=[group.loc[group.controller==k].iloc[0] for k in KINDS]
    for metric in ['rmse_actual_motion_rad','cartesian_rmse_motion_mm','TV_cmd','effort_cmd','slew_rms_cmd_Nm_s','max_ddq_violation','ddq_violation_duration_s']:
        detpaired.append(dict(profile=profile,case_name=case,block=a.block,metric=metric,flatness=a[metric],ltv=b[metric],paired_change_percent=np.nan if b[metric]==0 else 100*(a[metric]/b[metric]-1)))
pd.DataFrame(detpaired).to_csv(D/'efectos_deterministas.csv',index=False)
M.to_csv(D/'metricas_recibidas_232.csv',index=False)

initial_details=[];events=[]
for row in M.itertuples(index=False):
    if row.block=='initial':
        z=arrays[row.id];mask=z['solver'][:,1].astype(bool)
        initial_details.append(dict(id=row.id,fallback_ticks=int(mask.sum()),ticks=len(mask),
            first_fallback_time_s=np.nan if not mask.any() else z['tc'][np.flatnonzero(mask)[0]],
            solver_flag_counts=json.dumps({str(float(k)):int(n) for k,n in zip(*np.unique(z['solver'][:,0],return_counts=True))})))
    if row.block=='adaptation':
        ee=metas[row.id]['events'];ee=ee if isinstance(ee,list) else [ee]
        for i,event in enumerate(ee):
            events.append(dict(id=row.id,event_index_one_based=i+1,time_s=event['time'],status=event['info']['status'],
                candidates_tested=event['info']['tested'],old_torque_ratio=event['info']['old_ratio'],
                new_plan_duration_s=event['candidate']['T'],new_plan_end_time_s=event['time']+event['candidate']['T'],
                used_slope_joint2=event['used_estimate']['slope'][1]))
pd.DataFrame(initial_details).to_csv(D/'fallback_error_inicial.csv',index=False)
pd.DataFrame(events).to_csv(D/'eventos_supervisor.csv',index=False)

# All observed latencies. Include first update and every outlier.
timing=[];worst=[]
for scope in ['random','non_convergence','all_including_convergence']:
    subset=M.loc[M.block=='random'] if scope=='random' else M.loc[M.block!='convergence'] if scope=='non_convergence' else M
    for profile in PROFILES:
        for kind in KINDS:
            rr=subset.loc[(subset.profile==profile)&(subset.controller==kind)]
            times=np.concatenate([arrays[id_]['full_tick_s']*1000 for id_ in rr.id])
            steady=np.concatenate([arrays[id_]['full_tick_s'][1:]*1000 for id_ in rr.id])
            timing.append(dict(scope=scope,profile=profile,controller=kind,runs=len(rr),updates=len(times),
                preparation_mean_ms=rr.preparation_ms.mean(),preparation_min_ms=rr.preparation_ms.min(),preparation_max_ms=rr.preparation_ms.max(),
                first_mean_ms=rr.first_tick_ms.mean(),first_median_ms=rr.first_tick_ms.median(),first_max_ms=rr.first_tick_ms.max(),
                mean_run_p95_ms=rr.full_tick_p95_ms.mean(),pooled_p95_ms=np.quantile(times,.95,method='hazen'),
                maximum_ms=times.max(),max_excluding_first_ms=steady.max(),deadline_exceedances=int((times>20).sum())))
for row in M.itertuples(index=False):
    z=arrays[row.id];k=int(np.argmax(z['full_tick_s']))
    worst.append(dict(id=row.id,block=row.block,profile=row.profile,controller=row.controller,tick_zero_based=k,time_s=z['tc'][k],
        full_tick_ms=1000*z['full_tick_s'][k],governor_ms=1000*z['governor_s'][k],reference_ms=1000*z['reference_s'][k],
        observer_ms=1000*z['observer_s'][k],model_ms=1000*z['solver'][k,5],assembly_ms=1000*z['solver'][k,6],solver_ms=1000*z['solver'][k,7]))
pd.DataFrame(timing).to_csv(D/'tiempos_resumidos.csv',index=False)
pd.DataFrame(worst).to_csv(D/'peor_paso_por_ejecucion.csv',index=False)

# Convergence recomputed at the control grid; fine peak is received NPZ metric.
convergence=[]
for row in M.loc[M.block=='convergence'].itertuples(index=False):
    baseid=row.id.replace('_fine_','_');base=M.loc[M.id==baseid].iloc[0];a=arrays[row.id];b=arrays[baseid]
    convergence.append(dict(id=row.id,base_id=baseid,max_q_difference_rad=abs(a['x'][:6]-b['x'][:6]).max(),
        max_dq_difference_rad_s=abs(a['x'][6:]-b['x'][6:]).max(),rmse_difference_rad=abs(row.rmse_actual_motion_rad-base.rmse_actual_motion_rad),
        ddq_peak_difference=abs(row.max_ddq_violation-base.max_ddq_violation),same_goal=row.goal_reached==base.goal_reached))
CV=pd.DataFrame(convergence);CV.to_csv(D/'convergencia_recalculada.csv',index=False)
# Optional nominal path decomposition: interpolated cache, never actual torque.
parts=[];part_summary=[]
for profile in PROFILES:
    for case in ['healthy','limited_authority']:
        id_=f'{profile}_{case}_Observer';z=arrays[id_];m=metas[id_]['nominal_model'];phase=z['phase'];s,v,a=phase
        interp=lambda key:np.array([np.interp(s,z['path_s'],c) for c in z[key]])
        comp={'inertial':interp('path_A')*a,'quadratic':interp('path_C')*v**2,'gravity':interp('path_G'),
            'friction':np.array(m['fv'])[:,None]*z['path_d'][:,None]*v+np.array(m['fc'])[:,None]*np.tanh(15*z['path_d'][:,None]*v)}
        mask=s<1-1e-8
        for label,val in comp.items():
            part_summary.append(dict(profile=profile,case_name=case,component=label,
                rms_vector_norm_motion_Nm=np.sqrt(np.mean(np.sum(val[:,mask]**2,axis=0))),max_abs_joint2_motion_Nm=abs(val[1,mask]).max()))
        for k,t in enumerate(z['tc']):
            parts.append(dict(id=id_,time_s=t,phase=s[k],**{label+'_joint2_Nm':val[1,k] for label,val in comp.items()},
                sum_joint2_Nm=sum(val[1,k] for val in comp.values())))
pd.DataFrame(parts).to_csv(D/'descomposicion_nominal_interpolada.csv',index=False)
pd.DataFrame(part_summary).to_csv(D/'descomposicion_nominal_resumen.csv',index=False)

summary=[]
for (block,kind),rr in M.groupby(['block','controller'],sort=False):
    summary.append(dict(block=block,controller=kind,runs=len(rr),goals=int(rr.goal_reached.sum()),
        **{k+'_violations':int((rr['max_'+k+'_violation']>1e-6).sum()) for k in ['q','dq','ddq']},
        fallback_runs=int((rr.fallback_fraction>0).sum()),deadlines=int(rr.deadline_exceedances.sum())))
pd.DataFrame(summary).to_csv(D/'cumplimiento_por_bloque.csv',index=False)
pd.DataFrame(checks).to_csv(D/'comprobaciones_metricas.csv',index=False)
for p in [E/'frozen/all_metrics.csv',E/'frozen/protocol.json',E/'portable/index.json']:
    record(p)
(OUT/'MANIFEST_ENTRADAS.json').write_text(json.dumps(input_hashes,indent=2)+'\n')
receipt=dict(status='PASS',metric_comparisons=len(checks),runs=232,pairs=116,independent_random_pairs_per_path=30,
    paired_portable_inputs_equal=True,npz_hashes_checked=232,original_MAT_hashes_checked=len(list(RAW.glob('*.mat'))) if RAW else 0,
    fine_durations_recomputed_for_available_MAT=len(list(RAW.glob('*.mat'))) if RAW else 0,fine_peaks_all_runs_checked_against_portable_violation=True,
    other_fine_durations='All 232 MAT checked' if RAW else 'Frozen metrics; use --raw to recompute dense-grid durations and applied effort',
    simulations_run=False,input_origin='Locally generated MAT files; no downloaded run data',bootstrap='Descriptive percentile bootstrap, 10000 resamples; seeds 20261018 / 20261019; no BCa; no multiplicity adjustment',
    python=platform.python_version(),numpy=np.__version__,pandas=pd.__version__,scipy=scipy.__version__,script_sha256=sha(__file__))
(OUT/'RECIBO_VERIFICACION.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))
