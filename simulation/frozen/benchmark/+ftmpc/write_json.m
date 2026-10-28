function write_json(path,data)
folder=fileparts(path); if ~isfolder(folder), mkdir(folder); end
tmp=[tempname(folder) '.json']; fid=fopen(tmp,'w','n','UTF-8'); assert(fid>0);
cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(data,PrettyPrint=true)); clear cleanup;
movefile(tmp,path,'f');
end
