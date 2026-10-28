function manifest=manifest(repo)
%MANIFEST Hash code, kernels, protocol and external trajectory dependencies.
roots={fullfile(repo,'benchmark'),fullfile(repo,'benchmark_v3')}; entries={};
for z=1:numel(roots)
    files=dir(fullfile(roots{z},'**','*.m'));
    for k=1:numel(files)
        path=fullfile(files(k).folder,files(k).name);
        if contains(path,[filesep 'results' filesep]) || contains(path,[filesep 'generated' filesep]), continue; end
        entries{end+1}=entry(path,repo); %#ok<AGROW>
    end
end
extra={fullfile(repo,'benchmark_v3','PROTOCOL.md'),fullfile(repo,'benchmark','third_party','kinova_gen3_6dof.urdf'), ...
    fullfile(repo,'Code_FlatnessMPC','Trayectorias','InertiaCase2.mat'),fullfile(repo,'Code_FlatnessMPC','Trayectorias','GravityCase2.mat')};
mexfiles=dir(fullfile(repo,'benchmark','generated',['*.' mexext]));
for k=1:numel(mexfiles), extra{end+1}=fullfile(mexfiles(k).folder,mexfiles(k).name); end
for k=1:numel(extra), entries{end+1}=entry(extra{k},repo); end
entries=[entries{:}]; [~,ix]=sort({entries.path}); entries=entries(ix);
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(jsonencode(entries)));
manifest=struct('files',entries,'sha256',lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[])));
end
function e=entry(path,repo)
e=struct('path',erase(path,[repo filesep]),'sha256',ftmpc.hash_file(path));
end

