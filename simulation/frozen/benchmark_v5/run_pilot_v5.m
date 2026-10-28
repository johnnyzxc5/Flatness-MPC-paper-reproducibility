function run_pilot_v5
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'),fullfile(repo,'benchmark_v3'));
maxNumCompThreads(1); validation=validate_v5; c=ftmpc.config(); c.smooth_multiplier=.001; c.version='ftmpc-v5-pilot';
m=ftmpc.model(); o=ftv5.options();
cases=struct('name',{'Inertia','GravityReverse'},'profile',{'Inertia','Gravity'},'reverse',{false,true}, ...
    'Tnom',{4.5,3},'health',{.555,.50});
src=ftv3.manifest(repo); src.parent_v3_sha256=src.sha256;
files=dir(fullfile(root,'**','*.m')); files=files(~contains({files.folder},[filesep 'results']));
paths=[{fullfile(root,'PROTOCOL.md')},arrayfun(@(f)fullfile(f.folder,f.name),files,'UniformOutput',false)'];
for j=1:numel(paths)
    src.files(end+1)=struct('path',strrep(paths{j}(numel(repo)+2:end),'\','/'),'sha256',ftmpc.hash_file(paths{j}));
end
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(jsonencode(src.files)));
src.sha256=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
folder=fullfile(root,'results',['pilot_' src.sha256(1:12)]); if ~isfolder(folder), mkdir(folder); end
for j=1:numel(src.files)
    s=src.files(j); dest=fullfile(folder,'source_snapshot',s.path);
    if ~isfolder(fileparts(dest)), mkdir(fileparts(dest)); end
    if ~isfile(dest), copyfile(fullfile(repo,s.path),dest); end
    assert(strcmp(ftmpc.hash_file(dest),s.sha256));
end
protocol=struct('sources',src,'cases',cases,'config',c,'options',o,'validation',validation, ...
    'fault_shapes',{{'step','ramp_fast','ramp_slow'}},'fault_start',.5,'ramps',[0 .3 2], ...
    'runs',48,'environment',struct('matlab',version,'computer',computer,'threads',maxNumCompThreads));
ftmpc.write_json(fullfile(folder,'protocol.json'),protocol);
ftmpc.write_json(fullfile(root,'results','latest_pilot.json'),struct('folder',folder,'sha256',src.sha256));
rows=struct([]); fprintf('Pilot: %s\n',folder);
for i=1:numel(cases)
    cs=cases(i); tr=ftmpc.trajectory(cs.profile,'evaluation',m,c);
    if cs.reverse, old=tr.q0; tr.q0=tr.q1; tr.q1=old; tr.profile='GravityReverse'; end
    tr.t=0:c.Ts:cs.Tnom; [tr.q,tr.dq,tr.ddq]=ftmpc.reference(tr.t,tr,cs.Tnom);
    tr.tau=zeros(size(tr.q));
    for kk=1:numel(tr.t)
        [M,b]=ftmpc_dynamics_mex(tr.q(:,kk),tr.dq(:,kk),m); tr.tau(:,kk)=M*tr.ddq(:,kk)+b;
    end
    % Old sampled diagnostics referred to the historical timing/direction.
    tr=rmfield(tr,{'p','gravity','inertia','alpha_required','nominal_ratios'});
    tr.provenance='Historical endpoints; direction and duration explicitly recorded in v5 protocol.';
    path=ftv5.path_cache(tr,m,o); assert(cs.health>=path.floor(2));
    for shape=1:3
        fault=[.5,.5+protocol.ramps(shape),cs.health];
        for name={'HoldCorrected','LTV'}
            for treatment={'fixed','constant','trend','trend_governor'}
                id=sprintf('%s_%s_%s_%s',cs.name,protocol.fault_shapes{shape},name{1},treatment{1});
                file=fullfile(folder,[id '.mat']); meta=fullfile(folder,[id '.json']);
                if isfile(file) && isfile(meta)
                    receipt=jsondecode(fileread(meta)); assert(strcmp(ftmpc.hash_file(file),receipt.sha256));
                    loaded=load(file,'r'); r=loaded.r;
                else
                    r=ftv5.simulate(name{1},treatment{1},tr,cs.Tnom,fault,m,c,o); r.id=id;
                    tmp=[tempname(folder) '.mat']; save(tmp,'r','-v7'); movefile(tmp,file);
                    ftmpc.write_json(meta,struct('id',id,'sha256',ftmpc.hash_file(file),'metrics',r.metrics));
                end
                row=struct('id',id,'scenario',cs.name,'shape',protocol.fault_shapes{shape},'controller',name{1},'treatment',treatment{1});
                fields=fieldnames(r.metrics); for j=1:numel(fields), row.(fields{j})=r.metrics.(fields{j}); end
                rows=[rows;row]; %#ok<AGROW>
                fprintf('%s goal=%d t=%.2f rmse=%.4g bounds=[%.3g %.3g %.3g] replans=%d missed=%d wall=%.2f\n', ...
                    id,r.metrics.goal_reached,r.metrics.duration_s,r.metrics.rmse_adapted_rad, ...
                    r.metrics.max_q_violation,r.metrics.max_dq_violation,r.metrics.max_ddq_violation, ...
                    r.metrics.replans,r.metrics.deadline_exceedances,r.wall_s);
                writetable(struct2table(rows),fullfile(folder,'metrics.csv'));
            end
        end
    end
end
for j=1:numel(src.files)
    assert(strcmp(ftmpc.hash_file(fullfile(repo,src.files(j).path)),src.files(j).sha256),'Source changed during pilot');
end
ftmpc.write_json(fullfile(folder,'finished.json'),struct('status','ALL_48_RECORDED','rows',numel(rows)));
end
