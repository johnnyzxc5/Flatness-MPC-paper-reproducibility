function tr=trajectory(profile,split,m,c)
%TRAJECTORY Frozen historical endpoints; analytic quintic derivatives.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
d=load(fullfile(root,'Code_FlatnessMPC','Trayectorias',[profile 'Case2.mat']),'TrajectoryData');
q0=d.TrajectoryData.q(:,1); q1=d.TrajectoryData.q(:,end);
if strcmp(split,'calibration')
    % Deterministic distinct reference, selected without controller outcomes.
    q0=.85*q0; q1=.85*q1;
end
tr=struct('profile',profile,'split',split,'t',0:c.Ts:c.Tf,'q0',q0,'q1',q1);
[tr.q,tr.dq,tr.ddq]=ftmpc.reference(tr.t,tr,c.Tf);
n=numel(tr.t); tr.tau=zeros(6,n); tr.p=zeros(3,n); tr.gravity=zeros(6,n); tr.inertia=zeros(6,n);
for k=1:n
    [M,b]=ftmpc_dynamics_mex(tr.q(:,k),tr.dq(:,k),m);
    tr.tau(:,k)=M*tr.ddq(:,k)+b; tr.inertia(:,k)=M*tr.ddq(:,k);
    [~,tr.gravity(:,k)]=ftmpc_dynamics_mex(tr.q(:,k),zeros(6,1),m);
    tr.p(:,k)=ftmpc.fk(tr.q(:,k),m);
end
tr.alpha_required=abs(tr.tau)./m.taumax;
tr.nominal_ratios=[max(abs(tr.q)./m.qmax,[],'all'),max(abs(tr.dq)./m.dqmax,[],'all'), ...
    max(abs(tr.ddq)./m.ddqmax,[],'all'),max(abs(tr.tau)./m.taumax,[],'all')];
assert(max(tr.nominal_ratios)<1,'Archived trajectory violates corrected nominal limits');
tr.provenance='Historical endpoints, re-evaluated with manufacturer geometry; calibration endpoints scaled by 0.85.';
end
