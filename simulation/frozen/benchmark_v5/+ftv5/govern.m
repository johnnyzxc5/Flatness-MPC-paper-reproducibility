function [next,info]=govern(p,age,path,m,e,o,Tnom)
tm=tic; [s,v,a]=ftv5.phase(p,age); next=[];
info=struct('status','keep','tested',0,'seconds',0);
if s>=1-1e-8, info.seconds=toc(tm); return; end
[keep,ratio]=ftv5.check_plan(p,age,path,m,e,o,o.verify_dt); info.old_ratio=ratio;
% Nominal remaining travel time at this geometric position, independent of delay.
z=linspace(0,1,501); nominal_remaining=Tnom*(1-interp1(10*z.^3-15*z.^4+6*z.^5,z,s));
if keep && (p.T-age)<=1.10*nominal_remaining
    info.seconds=toc(tm); return;
end
[~,A,b]=ftv5.torque(path,s,v,0,m); limit=(1-o.reserve)*e.current.*m.taumax;
lo=-min(m.ddqmax./max(abs(path.d),1e-12)); hi=-lo;
for j=1:6
    if abs(A(j))<1e-10
        if abs(b(j))>limit(j), lo=1; hi=0; break; end
    else
        limits=sort([(-limit(j)-b(j))/A(j),(limit(j)-b(j))/A(j)]);
        lo=max(lo,limits(1)); hi=min(hi,limits(2));
    end
end
if lo<=hi
    anchors=unique([max(lo,min(hi,a)),max(lo,min(hi,0)),lo,(lo+hi)/2],'stable');
    durations=sort(unique([max(.2,nominal_remaining)*o.duration_factors,o.duration_extra]));
    for T=durations
        if T<max(.2,nominal_remaining), continue; end
        if keep && T>=.90*(p.T-age), break; end
        for aa=anchors
            candidate=ftv5.phase_plan(s,v,aa,T); info.tested=info.tested+1;
            if ftv5.check_plan(candidate,0,path,m,e,o,o.plan_dt) && ftv5.check_plan(candidate,0,path,m,e,o,o.verify_dt)
                next=candidate; info.status='replan'; info.seconds=toc(tm); return;
            end
        end
    end
end
if ~keep, info.status='search_exhausted'; end
info.seconds=toc(tm);
end
