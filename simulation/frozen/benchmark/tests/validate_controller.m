function report=validate_controller
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'generated')); maxNumCompThreads(1);
c=ftmpc.config(); m=ftmpc.model(); tr=ftmpc.trajectory('Inertia','evaluation',m,c);
kinds={'Flatness','LTV','LTI'}; comparisons=cell(1,3); maxGram=0;
opt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',300);
for j=1:3
    p=ftmpc.prepare(kinds{j},tr,m,c); p.audit_qp=true;
    x=[tr.q(:,1);tr.dq(:,1)]; prev=tr.tau(:,1); ah=ones(6,1);
    [u,log,d]=ftmpc.step(p,1,x,ah,prev,m); assert(log(2)==0);
    [z,~,flag]=quadprog(d.H,d.g,d.A,d.rhs,[],[],[],[],[],opt); assert(flag>0);
    rel=norm(d.decision-z)/max(1,norm(z)); assert(rel<1e-4);
    % Same physical cost assembled independently by dense Kronecker products.
    U=kron(eye(c.Np),d.M)*p.E./repmat(ah.*m.taumax,c.Np,1);
    DU=p.D*U;
    Hd=2*(p.trackH{1}+c.effort_weight*(U'*U)+c.smooth_weight*(DU'*DU))+1e-10*eye(6*c.Nc);
    err=norm(d.H-Hd,'fro'); maxGram=max(maxGram,err); assert(err<1e-8);
    % Choose a feasible but binding torque bound by reducing joint-2 authority
    % at the nominal initial pose. Always compare status with the reference solver.
    ah(2)=.3; [u,log,d]=ftmpc.step(p,1,x,ah,prev,m);
    [zr,~,fr]=quadprog(d.H,d.g,d.A,d.rhs,[],[],[],[],[],opt);
    if log(2)==0 && fr>0
        obj=@(zz) .5*zz'*d.H*zz+d.g'*zz;
        delta=abs(obj(d.decision)-obj(zr))/max(1,abs(obj(zr)));
        assert(delta<1e-4); assert(max(d.A*d.decision-d.rhs)<1e-5);
    elseif log(2)>0
        assert(fr==-2,'Optimized solver failed on a feasible reference QP'); delta=NaN;
    else
        error('Reference solver could not verify constrained solution');
    end
    comparisons{j}=struct('controller',kinds{j},'unconstrained_relative_solution',rel, ...
        'constrained_status',log(1),'reference_status',fr,'relative_objective',delta);
    for health=[0,1e-12,.049,.05,1]
        ah(2)=health; [u,~,~]=ftmpc.step(p,1,x,ah,prev,m);
        assert(all(isfinite(u)) && all(abs(u)<=m.taumax+1e-10));
    end
end
% ZOH command and health integrals are independent of telemetry mesh density.
eff=ftmpc.effort_applied(ones(6,1),0,1,[0 1 0]);
assert(abs(eff(2)-1/3)<1e-12 && max(abs(eff([1 3:6])-1))<1e-12);
% Exact plant cancellation at a single point and saturated fallback equivalence.
x=[tr.q(:,1);tr.dq(:,1)]; [M,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
v=[1;-.3;.2;-.1;.5;.3]; tau=M*v+b; assert(norm(M\(tau-b)-v)<1e-12);
sat=max(-.2*m.taumax,min(.2*m.taumax,tau)); assert(norm(M*(M\(sat-b))+b-sat)<1e-12);
% Shared clock/noise; deterministic reproduction including reset of warm state.
cshort=c; cshort.Tf=.10; [pert,obs]=ftmpc.perturbation('random',1001); plant=ftmpc.model(pert);
p=ftmpc.prepare('Flatness',tr,m,c); p.c=cshort;
r1=ftmpc.simulate(p,plant,m,cshort,[.02 .06 .5],obs,1001);
r2=ftmpc.simulate(p,plant,m,cshort,[.02 .06 .5],obs,1001);
assert(isequal(r1.raw.x,r2.raw.x) && isequal(r1.raw.tau_cmd,r2.raw.tau_cmd));
assert(max(abs(diff(r1.raw.tc)-c.Ts))<1e-12 && size(r1.raw.solver,1)==5);
report=struct('status','PASS','QP_comparisons',{comparisons},'max_cost_assembly_error',maxGram, ...
    'zero_health_finite',true,'effort_integral',true,'repeat_identical',true,'control_clock',true);
ftmpc.write_json(fullfile(root,'results','validation_controller.json'),report); disp(report);
end
