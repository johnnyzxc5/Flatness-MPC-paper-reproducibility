function r=local_reference(plan,age,path,m,c)
[s,v,a]=ftv5.phase(plan,age+(0:c.Np)*c.Ts);
r=struct('q',path.q0+path.d*s,'dq',path.d*v,'ddq',path.d*a,'s',s,'v',v);
% Exact nominal torque for controller linearizations; cache interpolation is
% used only by the governor and is independently validated.
r.tau=zeros(6,c.Np+1);
for k=1:c.Np+1
    [M,b]=ftmpc_dynamics_mex(r.q(:,k),r.dq(:,k),m); r.tau(:,k)=M*r.ddq(:,k)+b;
end
end
