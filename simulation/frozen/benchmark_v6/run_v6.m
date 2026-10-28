function run_v6
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'),fullfile(repo,'benchmark_v3'),fullfile(repo,'benchmark_v5'));
maxNumCompThreads(1); validation=validate_v6; m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001; c.version='ftmpc-v6';
o=ftv5.options(); o.max_time=10;
variants={'Flatness','StateMap','Observer','StateObserver','FastStateObserver','LTV','LTVObserver'};
src=ftv3.manifest(repo);
for sub={'benchmark_v5','benchmark_v6'}
    files=dir(fullfile(repo,sub{1},'**','*.m')); files=files(~contains({files.folder},[filesep 'results']));
    paths=[{fullfile(repo,sub{1},'PROTOCOL.md')},arrayfun(@(f)fullfile(f.folder,f.name),files,'UniformOutput',false)'];
    for j=1:numel(paths), src.files(end+1)=struct('path',strrep(paths{j}(numel(repo)+2:end),'\','/'),'sha256',ftmpc.hash_file(paths{j})); end
end
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(jsonencode(src.files)));
src.sha256=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
folder=fullfile(root,'results',['campaign_' src.sha256(1:12)]); if ~isfolder(folder), mkdir(folder); end
for j=1:numel(src.files)
    s=src.files(j); dest=fullfile(folder,'source_snapshot',s.path);
    if ~isfolder(fileparts(dest)), mkdir(fileparts(dest)); end
    if ~isfile(dest), copyfile(fullfile(repo,s.path),dest); end
    assert(strcmp(ftmpc.hash_file(dest),s.sha256));
end
ftmpc.write_json(fullfile(folder,'protocol.json'),struct('sources',src,'variants',{variants},'config',c,'options',o, ...
    'validation',validation,'runs',196,'environment',struct('matlab',version,'computer',computer,'threads',maxNumCompThreads)));
ftmpc.write_json(fullfile(root,'results','latest_campaign.json'),struct('folder',folder,'sha256',src.sha256));
fprintf('Campaign %s\n',folder); rows=struct([]);
for profile={'Inertia','GravityReverse','Transfer'}
    name=profile{1}; base='Inertia'; Tnom=4.5; hf=.65; ncase=12;
    if strcmp(name,'GravityReverse'), base='Gravity'; Tnom=3; hf=.60; end
    if strcmp(name,'Transfer'), hf=.70; ncase=4; end
    tr=ftmpc.trajectory(base,'evaluation',m,c);
    if strcmp(name,'GravityReverse'), old=tr.q0; tr.q0=tr.q1; tr.q1=old; end
    if strcmp(name,'Transfer'), tr.q0=.8*tr.q0; tr.q1=.8*tr.q1; end
    tr=struct('profile',name,'q0',tr.q0,'q1',tr.q1,'t',0:c.Ts:Tnom,'provenance','v6 protocol endpoints/direction/duration');
    [tr.q,tr.dq,tr.ddq]=ftmpc.reference(tr.t,tr,Tnom);
    for z=1:ncase
        [pert,obs]=ftmpc.perturbation('nominal',6000+z); seed=6000+z; offset=zeros(12,1);
        switch z
            case 1, label='nominal'; fault=[.5 .8 hf];
            case 2, label='mass20'; pert.mass(:)=.2; fault=[.5*Tnom .5*Tnom hf];
            case 3, label='friction25'; pert.fv(:)=.25; pert.fc(:)=.25; fault=[.7*Tnom .7*Tnom+.3 hf];
            case 4, label='initial'; offset=[.03*ones(6,1);.05*ones(6,1)]; fault=[20 21 1];
            otherwise
                seed=6000+z-4; label=sprintf('random%d',seed); [pert,obs]=ftmpc.perturbation('random',seed);
                obs.bias=0; obs.delay_ticks=0; obs.sigma_alpha=0;
                start=.5; if mod(z,4)>=2, start=.45*Tnom; end
                duration=0; if mod(z,2)==0, duration=2; end
                fault=[start start+duration hf];
        end
        plant=ftmpc.model(pert); nominalpath=ftv5.path_cache(tr,m,o); realpath=ftv5.path_cache(tr,plant,o);
        health=ones(6,1); health(2)=fault(3);
        static_ratio=max(abs(realpath.G)./(health.*plant.taumax),[],'all');
        assert(fault(3)>=nominalpath.floor(2) && static_ratio<1,'Prespecified case outside static scope; report instead of silently substituting');
        % Rotate method order to reduce systematic warm-up ordering effects.
        order=circshift(1:7,[0 mod(z-1,7)]);
        for jj=order
            kind=variants{jj}; id=sprintf('%s_%s_%s',name,label,kind);
            file=fullfile(folder,[id '.mat']); meta=fullfile(folder,[id '.json']);
            if isfile(file) && isfile(meta)
                rr=jsondecode(fileread(meta)); assert(strcmp(ftmpc.hash_file(file),rr.sha256)); data=load(file,'r'); r=data.r;
            else
                r=ftv6.simulate(kind,tr,Tnom,fault,m,plant,c,o,obs,seed,offset); r.id=id; r.static_ratio=static_ratio;
                tmp=[tempname(folder) '.mat']; save(tmp,'r','-v7'); movefile(tmp,file);
                ftmpc.write_json(meta,struct('id',id,'sha256',ftmpc.hash_file(file),'metrics',r.metrics,'static_ratio',static_ratio));
            end
            row=struct('id',id,'profile',name,'case_name',label,'seed',seed,'controller',kind);
            fields=fieldnames(r.metrics); for j=1:numel(fields), row.(fields{j})=r.metrics.(fields{j}); end
            rows=[rows;row]; %#ok<AGROW>
            fprintf('%s goal=%d RMSE=%.4g TV=%.3g limits=%.3g/%.3g/%.3g map=%.2f wall=%.2f\n', ...
                id,r.metrics.goal_reached,r.metrics.rmse_motion_rad,r.metrics.TV_cmd,r.metrics.max_q_violation, ...
                r.metrics.max_dq_violation,r.metrics.max_ddq_violation,r.metrics.state_map_fraction,r.wall_s);
            writetable(struct2table(rows),fullfile(folder,'metrics.csv'));
        end
    end
end
assert(numel(rows)==196);
for j=1:numel(src.files), assert(strcmp(ftmpc.hash_file(fullfile(repo,src.files(j).path)),src.files(j).sha256),'Source changed during campaign'); end
ftmpc.write_json(fullfile(folder,'finished.json'),struct('status','ALL_196_RECORDED','runs',numel(rows)));
end
