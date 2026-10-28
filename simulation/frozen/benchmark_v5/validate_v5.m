function report=validate_v5
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'),fullfile(repo,'benchmark_v3'));
maxNumCompThreads(1); m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001; o=ftv5.options();
tr=ftmpc.trajectory('Inertia','evaluation',m,c); path=ftv5.path_cache(tr,m,o);
% Reference stays on the exact same joint path, even after retiming.
phase=ftv5.phase_plan(.25,.17,-.03,6); [s,v,a]=ftv5.phase(phase,[0 .01 1 5 6]);
assert(abs(s(1)-.25)<1e-12 && abs(v(1)-.17)<1e-12 && abs(a(1)+.03)<1e-12);
assert(s(end)==1 && v(end)==0 && a(end)==0);
% Interpolated path inverse dynamics versus physical MEX at off-grid points.
maxerr=0;
for k=1:200
    ss=mod(k*sqrt(2),1); vv=.4*mod(k*sqrt(3),1); aa=.5*sin(k);
    torque=ftv5.torque(path,ss,vv,aa,m);
    [M,b]=ftmpc_dynamics_mex(path.q0+path.d*ss,path.d*vv,m);
    maxerr=max(maxerr,max(abs(torque-(M*path.d*aa+b))));
end
assert(maxerr<1e-3,'Path interpolation exceeds declared tolerance');
% Ramp slope then plateau; jump reset independent of any future schedule.
history=[];
for k=0:15
    h=ones(6,1); h(2)=1-.4*k*c.Ts; [history,e]=ftv5.predict(history,h,c.Ts,o,path.floor);
end
assert(abs(e.slope(2)+.4)<1e-10);
for k=1:o.plateau_samples, [history,e]=ftv5.predict(history,h,c.Ts,o,path.floor); end
assert(e.slope(2)==0);
h(2)=.6; [history,e]=ftv5.predict(history,h,c.Ts,o,path.floor);
assert(e.mode(2)==2 && e.slope(2)==0);
assert(all(ftv5.forecast(e,0)==e.current));
% Constant health must reproduce v3 physical QP and action for both methods.
baseplan=ftv5.phase_plan(0,0,0,5); p=ftv5.template(path,m,c); kk=61; age=(kk-1)*c.Ts;
r=ftv5.local_reference(baseplan,age,path,m,c); x=[r.q(:,1);r.dq(:,1)];
previous=r.tau(:,1); health=repmat([1;.65;1;1;1;1],1,c.Np);
errors=zeros(2,4); names={'HoldCorrected','LTV'};
for j=1:2
    pv3=ftv3.prepare(names{j},tr,m,c); pv3.audit_qp=true;
    [old,~,d0]=ftv3.step(pv3,kk,x,health(:,1),previous,m);
    [now,~,d1]=ftv5.step(p,names{j},r,x,health,previous,m);
    errors(j,:)=[max(abs(now-old)),max(abs(d0.H-d1.H),[],'all'),max(abs(d0.g-d1.g)),max(abs(d0.A-d1.A),[],'all')];
end
assert(max(errors,[],'all')<2e-5,'Local horizon changed constant-health formulation');
% Rolling cache preserves the formulation and avoids redundant LTV models.
r2=ftv5.local_reference(baseplan,age+c.Ts,path,m,c); x2=[r2.q(:,1);r2.dq(:,1)];
for j=1:2
    [~,~,~,cache]=ftv5.step(p,names{j},r,x,health,previous,m);
    pc=p; pc.cache=cache;
    [uncached,~,du]=ftv5.step(p,names{j},r2,x2,health,previous,m);
    [cached,~,dc,updated]=ftv5.step(pc,names{j},r2,x2,health,previous,m);
    assert(max(abs(uncached-cached))<1e-7 && max(abs(du.H-dc.H),[],'all')<1e-7);
    if strcmp(names{j},'LTV'), assert(updated.evaluations==1); end
end
% Stage-varying objective equals the direct command cost.
health=ftv5.forecast(struct('current',health(:,1),'slope',[0;-.2;0;0;0;0],'floor',path.floor,'horizon',.6),(0:c.Np-1)*c.Ts);
[~,~,dd]=ftv5.step(p,'HoldCorrected',r,x,health,previous,m);
z=.001*cos((1:6*c.Nc)');
for dz=[z,2*z,-z]
    physical=c.effort_weight*(norm(dd.U*dz+dd.ub)^2-norm(dd.ub)^2) ...
        +c.smooth_multiplier*(norm(dd.DU*dz+dd.db)^2-norm(dd.db)^2);
    qL=p.L{1}(p.ixq,:); err=p.O{1}(p.ixq,:)*x-reshape(r.q(:,2:end),[],1);
    tracking=(norm(qL*dz+err)^2-norm(err)^2)/c.qscale^2;
    assert(abs(.5*dz'*dd.H*dz+dd.g'*dz-physical-tracking)<1e-8);
end
report=struct('status','PASS','max_interpolated_torque_error_Nm',maxerr,'qp_equivalence_errors',errors,'operating_floor',path.floor);
disp(report);
end
