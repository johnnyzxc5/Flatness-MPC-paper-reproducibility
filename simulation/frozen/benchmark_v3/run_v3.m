function run_v3
%RUN_V2 Frozen calibration and confirmation; all adverse outcomes retained.
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated')); maxNumCompThreads(1);
validate_v3; c=ftmpc.config(); c.version='ftmpc-v3'; m=ftmpc.model();
names={'FlatnessV1','HoldCorrected','LTV'}; profiles={'Inertia','Gravity'};
src=ftv3.manifest(repo); folder=fullfile(root,'results',['campaign_' src.sha256(1:12)]);
if ~isfolder(folder), mkdir(folder); end
for j=1:numel(src.files)
    s=src.files(j); dest=fullfile(folder,'source_snapshot',s.path);
    if ~isfolder(fileparts(dest)), mkdir(fileparts(dest)); end
    if ~isfile(dest), copyfile(fullfile(repo,s.path),dest); end
    assert(strcmp(ftmpc.hash_file(dest),s.sha256));
end
protocol=struct('sources',src,'config',c,'variants',{names},'profiles',{profiles}, ...
    'grid',[.001 .003 .01 .03 .1 .3 1],'principal_runs',300,'calibration_runs',84,'auxiliary_runs',6, ...
    'selection','Minimum sum of prespecified calibration score per method; HoldCorrected is the only new architecture.', ...
    'environment',struct('matlab',version,'computer',computer,'toolboxes',ver,'computational_threads',maxNumCompThreads));
ftmpc.write_json(fullfile(folder,'protocol.json'),protocol);
ftmpc.write_json(fullfile(root,'results','latest_campaign.json'),struct('folder',folder,'sha256',src.sha256));
fprintf('Campaign: %s\n',folder); grid=protocol.grid; scores=zeros(3,numel(grid));
for a=1:2
    tr=ftmpc.trajectory(profiles{a},'calibration',m,c); fs=ftmpc.faults(tr);
    for j=1:3
        p=ftv3.prepare(names{j},tr,m,c);
        for f=[1 3]
            for w=1:numel(grid)
                cfg=c; cfg.smooth_multiplier=grid(w); p.c=cfg; [~,obs]=ftmpc.perturbation('nominal',1729);
                id=sprintf('CAL_p%d_%s_f%d_w%02d',a,names{j},f,w);
                r=execute(id,p,m,m,cfg,fs(f,:),obs,1729,folder);
                u=r.raw.tau_cmd(:,1:r.ticks_completed)./m.taumax;
                violations=(r.metrics.max_q_violation/min(m.qmax))^2+ ...
                    (r.metrics.max_dq_violation/min(m.dqmax))^2+(r.metrics.max_ddq_violation/min(m.ddqmax))^2;
                scores(j,w)=scores(j,w)+(r.metrics.rmse_q/.01)^2+.1*mean(diff(u,1,2).^2,'all')+ ...
                    1e6*(r.metrics.fallback_fraction+violations+~r.complete);
            end
        end
    end
end
[bestscore,best]=min(scores,[],2); weights=grid(best);
cal=struct('grid',grid,'scores',scores,'selected_weights',weights,'variants',{names}, ...
    'winner','HoldCorrected','best_scores',bestscore);
ftmpc.write_json(fullfile(folder,'calibration.json'),cal); disp(cal);
kinds=names; selected_weights=weights;
specs=make_specs(kinds); assert(numel(specs)==306); ftmpc.write_json(fullfile(folder,'run_matrix.json'),specs);
for a=1:2
    tr=ftmpc.trajectory(profiles{a},'evaluation',m,c); fs=ftmpc.faults(tr);
    save(fullfile(folder,['trajectory_' profiles{a} '.mat']),'tr','fs');
    for j=1:3
        cfg=c; cfg.smooth_multiplier=selected_weights(j); p=ftv3.prepare(kinds{j},tr,m,cfg);
        for z=1:numel(specs)
            s=specs{z}; if s.profile~=a || ~strcmp(s.controller,kinds{j}), continue; end
            [pert,obs]=ftmpc.perturbation(s.plant,s.seed,s.mass);
            if strcmp(s.block,'E3'), obs.bias=s.bias; obs.delay_ticks=s.delay; end
            plant=ftmpc.model(pert); cc=cfg; cc.h=s.h; p.c=cc;
            execute(s.id,p,plant,m,cc,fs(s.fault,:),obs,s.seed,folder);
        end
    end
end
ftmpc.write_json(fullfile(folder,'finished.json'),struct('status','ALL_CONFIGURATIONS_RECORDED','principal',300,'calibration',84,'auxiliary',6));
fprintf('FINISHED: 390 configurations recorded (inspect individual completion/status).\n');
end
function r=execute(id,p,plant,nom,c,f,obs,seed,folder)
path=fullfile(folder,[id '.mat']); meta=fullfile(folder,[id '.json']);
if isfile(path) && isfile(meta)
    receipt=jsondecode(fileread(meta)); assert(strcmp(receipt.mat_sha256,ftmpc.hash_file(path)),'Corrupt checkpoint');
    data=load(path,'r'); r=data.r;
    assert(strcmp(r.controller,p.variant) && r.seed==seed && isequaln(r.config,c) && isequaln(r.observation,obs) && ...
        isequaln(r.fault,f) && isequaln(r.plant,plant) && isequaln(r.reference.q0,p.tr.q0) && isequaln(r.reference.q1,p.tr.q1),'Checkpoint config differs');
    return;
end
r=ftv3.simulate(p,plant,nom,c,f,obs,seed); r.run_id=id;
tmp=[tempname(folder) '.mat']; save(tmp,'r','-v7'); movefile(tmp,path,'f');
ftmpc.write_json(meta,struct('run_id',id,'status','RECORDED','completed_duration',r.complete, ...
    'mat_sha256',ftmpc.hash_file(path),'controller',r.controller,'failure',r.failure,'metrics',r.metrics));
fprintf('%s complete=%d qRMSE=%.5g TV=%.3g fallback=%.3g p95=%.3gms wall=%.2fs\n',id,r.complete,r.metrics.rmse_q,r.metrics.TV_cmd,r.metrics.fallback_fraction,r.metrics.controller_p95_ms,r.wall_seconds);
end
function specs=make_specs(names)
specs={}; counter=0;
for a=1:2
 for j=1:3
  base=struct('profile',a,'controller',names{j},'plant','nominal','seed',1729,'mass',0,'bias',0,'delay',0,'fault',3,'h',.001,'block','','id','');
  for plant={'nominal','historical'}
   for f=1:4, s=base; s.block='E1'; s.plant=plant{1}; s.fault=f; append(s); end
  end
  for dm=[-.20 -.10 .05 .10 .20], s=base; s.block='E2'; s.plant='mass'; s.mass=dm; append(s); end
  for b=[-.10 -.05 .05 .10], s=base; s.block='E3'; s.bias=b; append(s); end
  for delay=[1 5], s=base; s.block='E3'; s.delay=delay; append(s); end
  s=base; s.block='E3'; s.bias=.05; s.delay=2; append(s);
  for seed=3001:3030, s=base; s.block='E4'; s.plant='random'; s.seed=seed; append(s); end
  s=base; s.block='CONV'; s.h=.0005; append(s);
 end
end
 function append(s)
  counter=counter+1; s.id=sprintf('%s_p%d_%s_%03d',s.block,s.profile,s.controller,counter); specs{end+1}=s;
 end
end

