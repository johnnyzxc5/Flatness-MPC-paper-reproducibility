function manifest=source_manifest(root)
files=[dir(fullfile(root,'**','*.m'));dir(fullfile(root,'third_party','*.urdf'));dir(fullfile(root,'generated','*.mexw64'))];
entries=struct('path',{},'sha256',{});
for k=1:numel(files)
    path=fullfile(files(k).folder,files(k).name);
    if contains(path,[filesep 'generated' filesep]) && endsWith(path,'.m'), continue; end
    entries(end+1)=struct('path',erase(path,[root filesep]),'sha256',ftmpc.hash_file(path)); %#ok<AGROW>
end
[~,ix]=sort({entries.path}); entries=entries(ix);
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(uint8(jsonencode(entries)));
manifest=struct('files',entries,'sha256',lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[])));
end
