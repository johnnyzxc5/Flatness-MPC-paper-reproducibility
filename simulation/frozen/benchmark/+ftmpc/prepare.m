function p=prepare(kind,tr,m,c)
%PREPARE Cache all trajectory/model invariants for every controller.
tm=tic; np=c.Np; nc=c.Nc; N=numel(tr.t);
p=struct('kind',kind,'c',c,'tr',tr,'options',optimoptions('quadprog', ...
    'Display','off','Algorithm','active-set', ...
    'ConstraintTolerance',c.qp_tol,'OptimalityTolerance',c.qp_tol,'MaxIterations',c.qp_max_iter));
p.warm=zeros(6*nc,1);
E=zeros(6*np,6*nc);
for i=1:np, E((i-1)*6+(1:6),(min(i,nc)-1)*6+(1:6))=eye(6); end
p.E=E; p.D=eye(6*np)-diag(ones(6*(np-1),1),-6);
p.hold=zeros(np,nc); for i=1:np, p.hold(i,min(i,nc))=1; end
p.diffhold=(eye(np)-diag(ones(np-1,1),-1))*p.hold;
p.holdgram=p.hold'*p.hold; p.diffgram=p.diffhold'*p.diffhold; p.holdcount=sum(p.hold,1)';
ixq=reshape((0:np-1)*12+(1:6)',[],1); ixv=ixq+6;
p.ixq=ixq; p.ixv=ixv; p.qmax=repmat(m.qmax,np,1); p.dqmax=repmat(m.dqmax,np,1);
p.accmax=repmat(m.ddqmax,np,1); p.taulim=repmat(m.taumax,np,1);
p.limits=[p.qmax;p.dqmax;p.accmax];
p.tr.x=[tr.q;tr.dq]; p.tr.x(:,end+1:end+np)=repmat(p.tr.x(:,end),1,np);
p.tr.q(:,end+1:end+np)=repmat(tr.q(:,end),1,np);
p.tr.tau(:,end+1:end+np)=repmat(tr.tau(:,end),1,np);
p.tr.ddq(:,end+1:end+np)=zeros(6,np);
p.O=cell(1,N-1); p.L=cell(1,N-1); p.offset=cell(1,N-1);
p.accL=cell(1,N-1); p.accO=cell(1,N-1); p.accC=cell(1,N-1);
if strcmp(kind,'Flatness')
    A=[eye(6) c.Ts*eye(6);zeros(6) eye(6)]; B=[.5*c.Ts^2*eye(6);c.Ts*eye(6)];
    [O,L,d]=ftmpc.lift(A,B,zeros(12,1),np,nc);
    p.O(:)={O}; p.L(:)={L}; p.offset(:)={d};
    p.accL(:)={E}; p.accO(:)={zeros(6*np,12)}; p.accC(:)={zeros(6*np,1)};
else
    nk=N+np;
    Ad=zeros(12,12,nk); Bd=zeros(12,6,nk); ds=zeros(12,nk);
    Ac=zeros(12,12,nk); Bc=zeros(12,6,nk); cc=zeros(12,nk);
    for k=1:nk
        if strcmp(kind,'LTI') && k>1
            Ad(:,:,k)=Ad(:,:,1); Bd(:,:,k)=Bd(:,:,1);
            Ac(:,:,k)=Ac(:,:,1); Bc(:,:,k)=Bc(:,:,1);
        else
            kk=min(k,size(p.tr.x,2));
            [Ad(:,:,k),Bd(:,:,k),ds(:,k),Ac(:,:,k),Bc(:,:,k),cc(:,k)]= ...
                ftmpc.linearize(p.tr.x(:,kk),p.tr.tau(:,kk),m,c.Ts);
        end
        % Both torque baselines receive the same known reference feedforward.
        % Error model is delta x+ = Ad delta x + Bd delta tau; LTI keeps only
        % its Jacobians fixed, not the reference equilibrium torque.
        kk=min(k,size(p.tr.x,2));
        kn=min(k+1,size(p.tr.x,2));
        ds(:,k)=p.tr.x(:,kn)-Ad(:,:,k)*p.tr.x(:,kk)-Bd(:,:,k)*p.tr.tau(:,kk);
        cc(:,k)=[p.tr.x(7:12,kk);p.tr.ddq(:,kk)]-Ac(:,:,k)*p.tr.x(:,kk)-Bc(:,:,k)*p.tr.tau(:,kk);
    end
    for k=1:N-1
        [O,L,d]=ftmpc.lift(Ad(:,:,k:k+np-1),Bd(:,:,k:k+np-1),ds(:,k:k+np-1),np,nc);
        ao=zeros(6*np,12); al=zeros(6*np,6*nc); ac=zeros(6*np,1);
        for j=1:np
            ix=(j-1)*6+(1:6);
            if j==1, ox=eye(12); lu=zeros(12,6*nc); cx=zeros(12,1);
            else, prev=(j-2)*12+(1:12); ox=O(prev,:); lu=L(prev,:); cx=d(prev); end
            ax=Ac(7:12,:,k+j-1); bu=Bc(7:12,:,k+j-1);
            ao(ix,:)=ax*ox; al(ix,:)=ax*lu+bu*E(ix,:); ac(ix)=ax*cx+cc(7:12,k+j-1);
        end
        p.O{k}=O; p.L{k}=L; p.offset{k}=d; p.accO{k}=ao; p.accL{k}=al; p.accC{k}=ac;
    end
end
% Invariant tracking Hessian, constraints and row scaling are reused.
p.trackH=cell(1,N-1); p.stateA=cell(1,N-1);
for k=1:N-1
    lq=p.L{k}(ixq,:); lv=p.L{k}(ixv,:);
    p.trackH{k}=lq'*lq/c.qscale^2;
    p.stateA{k}=[lq;lv;p.accL{k}]./p.limits;
end
p.prep_seconds=toc(tm);
end
