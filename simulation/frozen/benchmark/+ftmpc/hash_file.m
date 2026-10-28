function h=hash_file(path)
fid=fopen(path,'rb'); assert(fid>0); cleanup=onCleanup(@()fclose(fid)); bytes=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes); h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
