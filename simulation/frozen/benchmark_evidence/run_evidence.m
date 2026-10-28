function run_evidence
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'),fullfile(repo,'benchmark_v3'),fullfile(repo,'benchmark_v5'),fullfile(repo,'benchmark_v6'));
maxNumCompThreads(1); validation=validate_evidence; m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001;
o=ftv5.options(); o.max_time=10; variants={'Observer','LTVObserver'};
src=ftv3.manifest(repo);
for sub={'benchmark_v5','benchmark_v6','benchmark_evidence'}
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
ftmpc.write_json(fullfile(folder,'protocol.json'),struct('sources',src,'variants',{variants},'config',c,'options',o,'validation',validation,'runs',232,'environment',struct('matlab',version,'computer',computer,'threads',maxNumCompThreads)));
ftmpc.write_json(fullfile(root,'results','latest_campaign.json'),struct('folder',folder,'sha256',src.sha256));
rows=struct([]); fprintf('Campaign %s\n',folder);
for profile={'Inertia','GravityReverse'}
    name=profile{1}; base='Inertia'; Tnom=4.5; hf=.65;
    if strcmp(name,'GravityReverse'), base='Gravity'; Tnom=3; hf=.60; end
    tr=ftmpc.trajectory(base,'evaluation',m,c);
    if strcmp(name,'GravityReverse'), old=tr.q0; tr.q0=tr.q1; tr.q1=old; end
    tr=struct('profile',name,'q0',tr.q0,'q1',tr.q1,'t',0:c.Ts:Tnom,'provenance','fixed v6 endpoints; evidence protocol');
    [tr.q,tr.dq,tr.ddq]=ftmpc.reference(tr.t,tr,Tnom);
    for z=1:58
        zz=z; cc=c;
        if z>54, selected=[31 33 45 53]; zz=selected(z-54); cc.h=.0005; end
        seed=7000+zz; [pert,obs]=ftmpc.perturbation('nominal',seed); offset=zeros(12,1); fault=[.5 .8 hf];
        block='random'; label=sprintf('random%d',seed);
        if zz<=30
            [pert,obs]=ftmpc.perturbation('random',seed); obs.bias=0; obs.delay_ticks=0; obs.sigma_alpha=0;
            dur=[0 .3 2]; onset=[.5 .45*Tnom .7*Tnom]; start=onset(mod(floor((zz-1)/3),3)+1); fault=[start start+dur(mod(zz-1,3)+1) hf];
        elseif zz<=40
            block='fault'; label='healthy'; fault=[20 21 1];
            if zz>31
                j=zz-32; durations=[0 .3 2]; onset=[.2 .5 .8]*Tnom; start=onset(mod(j,3)+1); duration=durations(floor(j/3)+1);
                fault=[start start+duration hf]; label=sprintf('shape%d_onset%d',floor(j/3)+1,mod(j,3)+1);
            end
        elseif zz<=48
            block='sensitivity'; values=[-.2 -.1 .1 .2 .3 -.25 .25 .5]; value=values(zz-40);
            if zz<=45, pert.mass(:)=value; label=sprintf('mass%+d',round(value*100));
            else, pert.fv(:)=value; pert.fc(:)=value; label=sprintf('friction%+d',round(value*100)); end
        elseif zz<=52
            block='health_estimation'; labels={'bias_negative','bias_positive','delay40ms','noise01'}; label=labels{zz-48};
            if zz==49, obs.bias=-.03; elseif zz==50, obs.bias=.03; elseif zz==51, obs.delay_ticks=2; else, obs.sigma_alpha=.01; end
        elseif zz==53
            block='adaptation'; label='limited_authority'; final=.555; if strcmp(name,'GravityReverse'), final=.5; end; fault=[.5 .8 final];
        else
            block='initial'; label='initial'; offset=[.03*ones(6,1);.05*ones(6,1)]; fault=[20 21 1];
        end
        if z>54, block='convergence'; label=[label '_fine']; end
        plant=ftmpc.model(pert); realpath=ftv5.path_cache(tr,plant,o); health=ones(6,1); health(2)=fault(3);
        static_ratio=max(abs(realpath.G)./(health.*plant.taumax),[],'all');
        order=circshift(1:2,[0 mod(z,2)]);
        for jj=order
            kind=variants{jj}; id=sprintf('%s_%s_%s',name,label,kind); file=fullfile(folder,[id '.mat']); meta=fullfile(folder,[id '.json']);
            if isfile(file) && isfile(meta)
                rr=jsondecode(fileread(meta)); assert(strcmp(ftmpc.hash_file(file),rr.sha256)); data=load(file,'r'); r=data.r;
            else
                r=fte.simulate(kind,tr,Tnom,fault,m,plant,cc,o,obs,seed,offset); r.id=id; r.static_ratio=static_ratio; r.block=block;
                r.metrics.static_scope=static_ratio<1; r.metrics.preparation_s=r.preparation_s;
                tmp=[tempname(folder) '.mat']; save(tmp,'r','-v7'); movefile(tmp,file);
                ftmpc.write_json(meta,struct('id',id,'sha256',ftmpc.hash_file(file),'metrics',r.metrics,'static_ratio',static_ratio));
            end
            row=struct('id',id,'profile',name,'case_name',label,'block',block,'seed',seed,'controller',kind);
            fields=fieldnames(r.metrics); for j=1:numel(fields), row.(fields{j})=r.metrics.(fields{j}); end
            rows=[rows;row]; %#ok<AGROW>
            fprintf('%d %s goal=%d RMSE=%.4g ddq=%.4g wall=%.2f\n',numel(rows),id,r.metrics.goal_reached,r.metrics.rmse_motion_rad,r.metrics.max_ddq_violation,r.wall_s);
            writetable(struct2table(rows),fullfile(folder,'metrics.csv'));
        end
    end
end
assert(numel(rows)==232);
for j=1:numel(src.files), assert(strcmp(ftmpc.hash_file(fullfile(repo,src.files(j).path)),src.files(j).sha256),'Source changed during campaign'); end
ftmpc.write_json(fullfile(folder,'finished.json'),struct('status','ALL_232_RECORDED','runs',numel(rows)));
end
