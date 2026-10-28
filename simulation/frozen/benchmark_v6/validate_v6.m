function report=validate_v6
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'),fullfile(repo,'benchmark_v5'));
maxNumCompThreads(1); m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001; o=ftv5.options();
tr=ftmpc.trajectory('Inertia','evaluation',m,c); path=ftv5.path_cache(tr,m,o); p=ftv5.template(path,m,c);
r=ftv5.local_reference(ftv5.phase_plan(0,0,0,4.5),1,path,m,c); x=[r.q(:,1);r.dq(:,1)];
h=repmat([1;.65;1;1;1;1],1,c.Np); previous=r.tau(:,1); errors=[];
for pair={{'HoldCorrected','Flatness'},{'LTV','LTV'}}
    names=pair{1}; [u0,~,d0]=ftv5.step(p,names{1},r,x,h,previous,m);
    [u1,~,d1]=ftv6.step(p,names{2},r,x,h,previous,m);
    ee=max([max(abs(u1-u0)),max(abs(d1.H-d0.H),[],'all'),max(abs(d1.g-d0.g))]);
    assert(ee<1e-10); errors(end+1)=ee; %#ok<AGROW>
end
% Zero observer and inactive switch are exact baseline variants.
p.disturbance=zeros(6,1); [u0,~,~]=ftv6.step(p,'Flatness',r,x,h,previous,m);
for name={'Observer','StateMap','StateObserver'}
    [u,~,~,cache]=ftv6.step(p,name{1},r,x,h,previous,m); assert(max(abs(u-u0))<1e-10 && ~cache.state_active);
end
xx=x; xx(1)=xx(1)+.03; [~,~,~,cache]=ftv6.step(p,'StateMap',r,xx,h,previous,m); assert(cache.state_active);
p.cache=cache; xx(1)=x(1)+.015; [~,~,~,cache]=ftv6.step(p,'StateMap',r,xx,h,previous,m); assert(cache.state_active);
p.cache=cache; [~,~,~,cache]=ftv6.step(p,'StateMap',r,x,h,previous,m); assert(~cache.state_active); p=rmfield(p,'cache');
% Estimate sign, gate, boundedness, and known constant residual convergence.
v=ftv6.variant('Observer'); truth=[.5;1;.4;.1;.1;.1]; [M,~]=ftmpc_dynamics_mex(x(1:6),zeros(6,1),m);
prediction=x; prediction(7:12)=x(7:12)+c.Ts*(M\truth); estimate=zeros(6,1);
for k=1:100, [estimate,info]=ftv6.update_observer(estimate,x,prediction,x,ones(6,1),ones(6,1),m,c,v); end
assert(info.updated && max(abs(estimate-truth))<1e-6);
[frozen,info]=ftv6.update_observer(estimate,x,prediction,x,.5*ones(6,1),ones(6,1),m,c,v);
assert(~info.updated && isequal(frozen,estimate));
prediction(7:12)=prediction(7:12)+100;
for k=1:100, estimate=ftv6.update_observer(estimate,x,prediction,x,ones(6,1),ones(6,1),m,c,v); end
assert(all(abs(estimate)<=.15*m.taumax));
% Physical command map includes estimated torque BEFORE the constraints.
p.disturbance=truth; [u,log,dd]=ftv6.step(p,'Observer',r,x,h,previous,m); assert(log(2)==0);
command=reshape(dd.U*dd.decision+dd.ub,6,[]).*m.taumax;
assert(max(abs(command(:,1)-u))<1e-9 && max(abs(command)./m.taumax,[],'all')<=1+1e-6);
% Independent convex solver checks the disturbed-QP action.
op=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','OptimalityTolerance',1e-9,'ConstraintTolerance',1e-9);
[z,~,flag]=quadprog(dd.H,dd.g,dd.A,dd.rhs,[],[],[],[],[],op); assert(flag>0);
assert(max(abs(dd.M*z(1:6)+dd.bias-h(:,1).*u))<1e-4);
% Internal coarsening is checked separately from tracking outcomes.
largest=0;
for k=1:30
    rr=ftv5.local_reference(ftv5.phase_plan(0,0,0,4.5),.1*k,path,m,c); xx=[rr.q(:,1);rr.dq(:,1)];
    [a,~,~]=ftv6.step(p,'StateObserver',rr,xx,h,rr.tau(:,1),m);
    [b,~,~]=ftv6.step(p,'FastStateObserver',rr,xx,h,rr.tau(:,1),m);
    largest=max(largest,max(abs(a-b)));
end
assert(largest<1e-4);
report=struct('status','PASS','base_equivalence',errors,'coarsening_max_command_Nm',largest,'observer_sign_gate_bound','PASS','independent_QP','PASS');
disp(report);
end
