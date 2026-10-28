function [pass,ratio]=check_plan(p,age,path,m,estimate,o,dt)
t=unique([min(age,p.T):dt:p.T,p.T]); [s,v,a]=ftv5.phase(p,t);
if min(s)<-1e-8 || max(s)>1+1e-8 || min(v)<-1e-8
    pass=false; ratio=Inf; return;
end
if max(abs(path.d*v)./m.dqmax,[],'all')>1+1e-9 || max(abs(path.d*a)./m.ddqmax,[],'all')>1+1e-9
    pass=false; ratio=Inf; return;
end
tau=ftv5.torque(path,s,v,a,m); h=ftv5.forecast(estimate,t-age);
ratio=max(abs(tau)./(h.*m.taumax),[],'all'); pass=ratio<=1-o.reserve+1e-9;
end
