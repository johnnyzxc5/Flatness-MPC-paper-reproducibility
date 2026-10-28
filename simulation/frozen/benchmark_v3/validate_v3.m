function validate_v3
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated')); maxNumCompThreads(1);
c=ftmpc.config(); c.smooth_multiplier=.03; m=ftmpc.model(); tr=ftmpc.trajectory('Inertia','calibration',m,c);
opt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','MaxIterations',500,'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
records={}; ir=0;
for name={'HoldCorrected'}
    p=ftv3.prepare(name{1},tr,m,c); p.audit_qp=true;
    for k=[1 60 120]
        x=[tr.q(:,k);tr.dq(:,k)]; prev=tr.tau(:,k);
        for health=[1 .4]
            ah=ones(6,1); ah(2)=health;
            [u,log,d]=ftv3.step(p,k,x,ah,prev,m);
            [z,~,flag]=quadprog(d.H,d.g,d.A,d.rhs,[],[],[],[],[],opt);
            assert(log(2)==0 && flag>0,'Validation QP unexpectedly infeasible');
            obj=@(zz) .5*zz'*d.H*zz+d.g'*zz;
            rel=abs(obj(d.decision)-obj(z))/max(1,abs(obj(z))); assert(rel<1e-5);
            assert(max(d.A*d.decision-d.rhs)<1e-5 && all(abs(u)<=m.taumax+1e-10));
            % Difference of full physical cost at two arbitrary decisions.
            za=d.decision; zb=za+.01*sin((1:numel(za))');
            un=p.O{k}*x+p.offset{k}; target=reshape(p.tr.q(:,k+1:k+c.Np),[],1);
            full=@(zz) sum((p.L{k}(p.ixq,:)*zz+un(p.ixq)-target).^2)/c.qscale^2+ ...
                c.effort_weight*sum((d.U*zz+d.ub).^2)+c.smooth_multiplier*sum((d.DU*zz+d.db).^2)+.5e-10*sum(zz.^2);
            err=abs((full(za)-full(zb))-(obj(za)-obj(zb))); assert(err<1e-8);
            if strcmp(name{1},'HoldCorrected')
                [M,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),m); acc=M\(ah.*u-b);
                assert(max(abs(acc)-m.ddqmax)<1e-4);
            end
            ir=ir+1; records{ir}=struct('kind',name{1},'tick',k,'health',health,'relative_objective',rel,'cost_error',err);
        end
    end
    for health=[0 1e-12 .049 .05]
        ah=ones(6,1); ah(2)=health; [u,~,~]=ftv3.step(p,1,[tr.q(:,1);tr.dq(:,1)],ah,tr.tau(:,1),m);
        assert(all(isfinite(u)) && all(abs(u)<=m.taumax+1e-10));
    end
end
% Identical plant/clock/random streams and legacy actions under the v2 runner.
cshort=c; cshort.Tf=.1; [pert,obs]=ftmpc.perturbation('random',2001); plant=ftmpc.model(pert);
for name={'FlatnessV1','LTV'}
    p=ftv3.prepare(name{1},tr,m,c); p.c=cshort;
    r=ftv3.simulate(p,plant,m,cshort,[.02 .06 .5],obs,2001);
    r0=ftmpc.simulate(p,plant,m,cshort,[.02 .06 .5],obs,2001);
    assert(isequal(r.raw.x,r0.raw.x) && isequal(r.raw.tau_cmd,r0.raw.tau_cmd));
end
p=ftv3.prepare('HoldCorrected',tr,m,c); p.c=cshort;
r1=ftv3.simulate(p,plant,m,cshort,[.02 .06 .5],obs,2001);
r2=ftv3.simulate(p,plant,m,cshort,[.02 .06 .5],obs,2001);
assert(isequal(r1.raw.x,r2.raw.x));
% Held-torque inverse: one-step errors near the declared reference family.
eq=zeros(50,2); ev=zeros(50,2); rng(613); p=ftv3.prepare('HoldCorrected',tr,m,c);
for z=1:50
    k=randi(250); x=[tr.q(:,k);tr.dq(:,k)]+.001*randn(12,1); v=tr.ddq(:,k)+.01*randn(6,1);
    pred=[x(1:6)+c.Ts*x(7:12)+.5*c.Ts^2*v;x(7:12)+c.Ts*v];
    for j=1:2
        [M,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),m); tau=M*v+b;
        if j==2
            ta=M*tr.ddq(:,k)+b; xa=ftmpc_integrate_mex(x,ta,0,.001,20,m,[10;11;1]);
            aa=(xa(7:12,end)-x(7:12))/c.Ts; tau=ta+p.K(:,:,k)*(v-aa);
        end
        xs=ftmpc_integrate_mex(x,tau,0,.0005,40,m,[10;11;1]);
        eq(z,j)=norm(xs(1:6,end)-pred(1:6)); ev(z,j)=norm(xs(7:12,end)-pred(7:12));
    end
end
report=struct('status','PASS','QP_checks',{records},'legacy_runner_equal',true,'zero_health_finite',true, ...
    'deterministic',true,'hold_position_error_mean_rad',mean(eq),'hold_velocity_error_mean_rad_s',mean(ev), ...
    'hold_velocity_error_max_rad_s',max(ev),'prediction_columns',{{'beginning_map','hold_corrected_map'}});
folder=fullfile(root,'results'); if ~isfolder(folder), mkdir(folder); end
ftmpc.write_json(fullfile(folder,'validation.json'),report); disp(report);
end

