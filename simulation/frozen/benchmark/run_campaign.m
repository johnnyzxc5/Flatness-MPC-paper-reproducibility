function run_campaign
%RUN_CAMPAIGN Reproducible serial calibration + 300 primary + 18 auxiliary.
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'generated'),fullfile(root,'tests'));
maxNumCompThreads(1);
validate_core; validate_controller; c=ftmpc.config(); m=ftmpc.model(); kinds={'Flatness','LTV','LTI'}; profiles={'Inertia','Gravity'};
src=ftmpc.source_manifest(root); folder=fullfile(root,'results',['campaign_' src.sha256(1:12)]);
if ~isfolder(folder), mkdir(folder); end
protocol=struct('config',c,'sources',src,'profiles',{profiles},'controllers',{kinds}, ...
    'calibration_grid',[.03 .1 .3 1 3 10 30],'calibration_score', ...
    '(rmse_q/0.01)^2 + 0.1*mean(normalized torque increment squared) + 1e6*(fallback_fraction + normalized squared maximum state violations + incomplete)', ...
    'principal_runs',300,'auxiliary_runs',18,'calibration_runs',42, ...
    'integration','RK4 held physical command; 1ms integration and logging', ...
    'timing','Serial run timings include cold start; warmed replay is reported separately', ...
    'environment',struct('matlab',version,'computer',computer,'toolboxes',ver,'computational_threads',maxNumCompThreads));
ftmpc.write_json(fullfile(folder,'protocol.json'),protocol);
ftmpc.write_json(fullfile(root,'results','latest_campaign.json'),struct('folder',folder,'sha256',src.sha256));
fprintf('Campaign folder: %s\n',folder);
trs=cell(1,2); faults=cell(1,2);
for a=1:2
    trs{a}=ftmpc.trajectory(profiles{a},'evaluation',m,c); faults{a}=ftmpc.faults(trs{a});
    tr=trs{a}; fs=faults{a}; save(fullfile(folder,['trajectory_' profiles{a} '.mat']),'tr','fs');
end
% Calibration is a fixed grid on distinct trajectories, before any test runs.
grid=protocol.calibration_grid; scores=zeros(3,numel(grid));
for a=1:2
    tr=ftmpc.trajectory(profiles{a},'calibration',m,c);
    for j=1:3
        p=ftmpc.prepare(kinds{j},tr,m,c);
        for w=1:numel(grid)
            cfg=c; cfg.smooth_multiplier=grid(w); p.c=cfg;
            [~,obs]=ftmpc.perturbation('nominal',1);
            id=sprintf('CAL_%s_%s_w%02d',profiles{a},kinds{j},w);
            r=execute(id,p,m,m,cfg,[10 11 1],obs,1729,folder);
            u=r.raw.tau_cmd./m.taumax;
            violations=(r.metrics.max_q_violation/min(m.qmax))^2+ ...
                (r.metrics.max_dq_violation/min(m.dqmax))^2+(r.metrics.max_ddq_violation/min(m.ddqmax))^2;
            scores(j,w)=scores(j,w)+(r.metrics.rmse_q/.01)^2+.1*mean(diff(u,1,2).^2,'all')+ ...
                1e6*(r.metrics.fallback_fraction+violations+~r.complete);
        end
    end
end
[~,best]=min(scores,[],2); weights=grid(best);
ftmpc.write_json(fullfile(folder,'calibration.json'),struct('grid',grid,'scores',scores,'selected',weights,'controllers',{kinds}));
% All test configurations are fixed from this point onward.
specs=make_specs; assert(numel(specs)==318);
ftmpc.write_json(fullfile(folder,'run_matrix.json'),specs);
for a=1:2
    tr=trs{a}; fs=faults{a};
    for j=1:3
        cfg=c; cfg.smooth_multiplier=weights(j); p=ftmpc.prepare(kinds{j},tr,m,cfg);
        for z=1:numel(specs)
            s=specs{z}; if s.profile~=a || s.controller~=j, continue; end
            [pert,obs]=ftmpc.perturbation(s.plant,s.seed,s.mass);
            if strcmp(s.block,'E3'), obs.bias=s.bias; obs.delay_ticks=s.delay; end
            plant=ftmpc.model(pert); cc=cfg; cc.h=s.h; cc.smooth_multiplier=cfg.smooth_multiplier*s.weight;
            p.c=cc; id=s.id;
            execute(id,p,plant,m,cc,fs(s.fault,:),obs,s.seed,folder);
        end
    end
end
ftmpc.write_json(fullfile(folder,'finished.json'),struct('status','COMPLETE','principal',300,'calibration',42,'auxiliary',18));
fprintf('FINISHED: 360 configurations saved.\n');
end

function r=execute(id,p,plant,nom,c,f,obs,seed,folder)
path=fullfile(folder,[id '.mat']); meta=fullfile(folder,[id '.json']);
if isfile(path) && isfile(meta)
    receipt=jsondecode(fileread(meta));
    if strcmp(receipt.mat_sha256,ftmpc.hash_file(path))
        d=load(path,'r'); r=d.r; return;
    end
    error('Corrupt checkpoint: %s',id);
end
r=ftmpc.simulate(p,plant,nom,c,f,obs,seed); r.run_id=id;
tmp=[tempname(folder) '.mat']; save(tmp,'r','-v7'); movefile(tmp,path,'f');
ftmpc.write_json(meta,struct('run_id',id,'status','RECORDED','completed_duration',r.complete, ...
    'mat_sha256',ftmpc.hash_file(path),'metrics',r.metrics));
fprintf('%s complete=%d qRMSE=%.5g TV=%.3g fallback=%.3g wall=%.2fs\n',id,r.complete,r.metrics.rmse_q,r.metrics.TV_cmd,r.metrics.fallback_fraction,r.wall_seconds);
end

function specs=make_specs
specs={}; counter=0;
for a=1:2
 for j=1:3
  base=struct('profile',a,'controller',j,'plant','nominal','seed',1729,'mass',0,'bias',0,'delay',0,'fault',3,'h',.001,'weight',1,'block','','id','');
  for plant={'nominal','historical'}
   for f=1:4
    s=base; s.block='E1'; s.plant=plant{1}; s.fault=f; append(s);
   end
  end
  for dm=[-.20 -.10 .05 .10 .20]
   s=base; s.block='E2'; s.plant='mass'; s.mass=dm; append(s);
  end
  for b=[-.10 -.05 .05 .10]
   s=base; s.block='E3'; s.bias=b; append(s);
  end
  for delay=[1 5]
   s=base; s.block='E3'; s.delay=delay; append(s);
  end
  s=base; s.block='E3'; s.bias=.05; s.delay=2; append(s);
  for seed=1:30
   s=base; s.block='E4'; s.plant='random'; s.seed=1000+seed; append(s);
  end
  s=base; s.block='CONV'; s.h=.0005; append(s);
  for w=[.3 3]
   s=base; s.block='ABL'; s.weight=w; append(s);
  end
 end
end
 function append(s)
  counter=counter+1; s.id=sprintf('%s_p%d_c%d_%03d',s.block,s.profile,s.controller,counter); specs{end+1}=s;
 end
end
