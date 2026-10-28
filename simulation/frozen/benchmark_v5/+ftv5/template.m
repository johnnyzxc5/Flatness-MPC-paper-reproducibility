function p=template(path,m,c)
tr=struct('t',[0 c.Ts],'q',repmat(path.q0,1,2),'dq',zeros(6,2),'ddq',zeros(6,2),'tau',zeros(6,2));
p=ftmpc.prepare('Flatness',tr,m,c);
end
