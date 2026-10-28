function report=validate_core
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'generated'));
m=ftmpc.model(); warning('off','robotics:robotmanip:urdf:MeshFileNotFound');
r=importrobot(fullfile(root,'third_party','kinova_gen3_6dof.urdf'),'DataFormat','column'); r.Gravity=[0 0 -9.81];
rng(1729); errors=zeros(20,5);
for k=1:20
    q=(rand(6,1)-.5)*2; dq=randn(6,1)*.3;
    [M,b]=ftmpc_dynamics_mex(q,dq,m); [Mm,bm]=ftmpc_dynamics(q,dq,m);
    errors(k,1)=norm(M-(massMatrix(r,q)+diag(m.rotor)),'fro');
    br=inverseDynamics(r,q,dq,zeros(6,1))+m.fv.*dq+m.fc.*tanh(15*dq);
    errors(k,2)=norm(b-br); errors(k,3)=norm(M-Mm,'fro')+norm(b-bm);
    [p,~]=ftmpc.fk(q,m); T=getTransform(r,q,'end_effector_link'); errors(k,4)=norm(p-T(1:3,4));
    [~,g]=ftmpc_dynamics_mex(q,zeros(6,1),m); grad=zeros(6,1); e=1e-6;
    for j=1:6
        qp=q; qm=q; qp(j)=qp(j)+e; qm(j)=qm(j)-e;
        [~,up]=ftmpc.fk(qp,m); [~,um]=ftmpc.fk(qm,m); grad(j)=(up-um)/(2*e);
    end
    errors(k,5)=norm(g-grad); assert(min(eig(M))>0);
end
assert(max(errors,[],'all')<1e-6,'Dynamics/FK validation failed');
maxLift=0;
for vary=0:1
    A=randn(4,4,1+vary*6)*.1; B=randn(4,2,1+vary*6); d=randn(4,1+vary*6);
    [O,L,c]=ftmpc.lift(A,B,d,7,3); x0=randn(4,1); u=randn(2,3); x=x0; direct=zeros(28,1);
    for i=1:7
        x=A(:,:,min(i,size(A,3)))*x+B(:,:,min(i,size(B,3)))*u(:,min(i,3))+d(:,min(i,size(d,2)));
        direct((i-1)*4+(1:4))=x;
    end
    maxLift=max(maxLift,norm(direct-(O*x0+L*u(:)+c)));
end
assert(maxLift<1e-11);
report=struct('status','PASS','random_states',20,'max_errors',max(errors), ...
    'error_columns',{{'mass_RBT','bias_RBT','MEX_MATLAB','FK_RBT','gravity_potential'}},'lift_error',maxLift);
ftmpc.write_json(fullfile(root,'results','validation_core.json'),report); disp(report);
end
