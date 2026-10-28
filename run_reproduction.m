function run_reproduction(action)
%RUN_REPRODUCTION Isolated build, validation, smoke or final 232-run campaign.
if nargin<1, action='validate'; end
assert(ismember(action,{'build','identity','validate','smoke','campaign'}),'Unknown action');
root=fileparts(mfilename('fullpath')); source=fullfile(root,'simulation','frozen');
workspace=fullfile(root,'.runtime','simulation_workspace');
if ~isfolder(workspace), mkdir(workspace); end
entries=dir(fullfile(source,'**','*')); entries=entries(~[entries.isdir]);
for j=1:numel(entries)
    from=fullfile(entries(j).folder,entries(j).name);
    rel=from(numel(source)+2:end); to=fullfile(workspace,rel);
    if ~isfolder(fileparts(to)), mkdir(fileparts(to)); end
    if ~isfile(to), copyfile(from,to); end
end
% Generate the historical-layout trajectory inputs from explicit endpoint settings.
endpoints=jsondecode(fileread(fullfile(root,'config','trajectory_endpoints.json')));
trajectoryFolder=fullfile(workspace,'Code_FlatnessMPC','Trayectorias');
if ~isfolder(trajectoryFolder), mkdir(trajectoryFolder); end
for profile={'Inertia','Gravity'}
    name=profile{1}; values=endpoints.(name);
    expected=[values.q0_rad(:),values.q1_rad(:)];
    target=fullfile(trajectoryFolder,[name 'Case2.mat']);
    if isfile(target)
        previous=load(target,'TrajectoryData');
        assert(isequal(previous.TrajectoryData.q(:,[1 end]),expected),'Staged endpoints changed; choose a fresh runtime workspace');
    else
        TrajectoryData=struct('q',expected); %#ok<NASGU>
        save(target,'TrajectoryData','-v7');
    end
end
addpath(fullfile(workspace,'benchmark'),fullfile(workspace,'benchmark','generated'), ...
    fullfile(workspace,'benchmark','tests'),fullfile(workspace,'benchmark_v3'), ...
    fullfile(workspace,'benchmark_v5'),fullfile(workspace,'benchmark_v6'),fullfile(workspace,'benchmark_evidence'));
maxNumCompThreads(1);
if strcmp(action,'build'), build_benchmark; return; end
assert(exist('ftmpc_dynamics_mex','file')==3 && exist('ftmpc_integrate_mex','file')==3, ...
    'Build the kernels on this platform first: run_reproduction(''build'')');
if strcmp(action,'campaign'), run_evidence; return; end
if strcmp(action,'identity')
    actual=ftv3.manifest(workspace);
    for sub={'benchmark_v5','benchmark_v6','benchmark_evidence'}
        files=dir(fullfile(workspace,sub{1},'**','*.m'));
        files=files(~contains({files.folder},[filesep 'results']));
        paths=[{fullfile(workspace,sub{1},'PROTOCOL.md')},arrayfun(@(f)fullfile(f.folder,f.name),files,'UniformOutput',false)'];
        for j=1:numel(paths)
            actual.files(end+1)=struct('path',strrep(paths{j}(numel(workspace)+2:end),'\','/'),'sha256',ftmpc.hash_file(paths{j}));
        end
    end
    md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(jsonencode(actual.files)));
    identity=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
    report=struct('status','PASS','source_files',numel(actual.files),'campaign_sha256',identity, ...
        'endpoint_inputs_generated',true,'historical_binary_identity_required',false);
    ftmpc.write_json(fullfile(root,'results','campaign_identity_validation.json'),report);
    disp(report); return;
end
if strcmp(action,'validate')
    core=validate_core; evidence=validate_evidence;
    report=struct('status','PASS','action',action,'matlab',version,'core',core,'evidence',evidence);
else
    m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001;
    o=ftv5.options(); o.max_time=10; [~,obs]=ftmpc.perturbation('nominal',7000);
    rows=struct([]);
    for profile={'Inertia','GravityReverse'}
        name=profile{1}; base='Inertia'; Tnom=4.5; health=.65;
        if strcmp(name,'GravityReverse'), base='Gravity'; Tnom=3; health=.60; end
        tr=ftmpc.trajectory(base,'evaluation',m,c);
        if strcmp(name,'GravityReverse'), q0=tr.q0; tr.q0=tr.q1; tr.q1=q0; end
        tr=struct('profile',name,'q0',tr.q0,'q1',tr.q1,'t',0:c.Ts:Tnom,'provenance','frozen endpoints; smoke validation');
        [tr.q,tr.dq,tr.ddq]=ftmpc.reference(tr.t,tr,Tnom);
        for test={'healthy','early_step'}
            label=test{1}; fault=[20 21 1]; if strcmp(label,'early_step'), fault=[.2*Tnom .2*Tnom health]; end
            for controller={'Observer','LTVObserver'}
                namec=controller{1}; r=fte.simulate(namec,tr,Tnom,fault,m,m,c,o,obs,7000,zeros(12,1));
                assert(r.ticks==500 && all(isfinite(r.raw.x),'all'));
                assert(r.metrics.goal_reached && all(abs(r.raw.u)<=m.taumax+1e-8,'all'));
                row=struct('profile',name,'case',label,'controller',namec,'ticks',r.ticks, ...
                    'goal_reached',r.metrics.goal_reached,'rmse_rad',r.metrics.rmse_motion_rad,'TV_cmd',r.metrics.TV_cmd, ...
                    'max_ddq_violation',r.metrics.max_ddq_violation);
                rows=[rows;row]; %#ok<AGROW>
                fprintf('%s %s %s goal=%d RMSE=%.8g\n',name,label,namec,row.goal_reached,row.rmse_rad);
            end
        end
    end
    report=struct('status','PASS','action',action,'matlab',version,'runs',rows);
end
ftmpc.write_json(fullfile(root,'results',['matlab_' action '_validation.json']),report);
fprintf('PASS: %s (reference data unchanged).\n',action);
end
