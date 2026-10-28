function [u,log,d]=step(p,k,x,ahat,previous,m)
if ~strcmp(p.variant,'HoldCorrected'), [u,log,d]=ftmpc.step(p,k,x,ahat,previous,m); return; end
total=tic; mapping=tic; c=p.c; np=c.Np; nc=c.Nc;
ahat=max(c.alpha_floor,min(1,ahat));
[M0,b0]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
va=p.tr.ddq(:,k); ta=M0*va+b0;
xs=ftmpc_integrate_mex(x,ta,0,.001,round(c.Ts/.001),m,[10;11;1]); xe=xs(:,end);
aa=(xe(7:12)-x(7:12))/c.Ts;
M=p.K(:,:,k); b=ta-M*aa;
N=M./(ahat.*m.taumax); bb=b./(ahat.*m.taumax);
U=kron(p.hold,N); ub=repmat(bb,np,1);
% Only first-interval endpoints: their anchor states are known before the QP.
[Me,be]=ftmpc_dynamics_mex(xe(1:6),xe(7:12),m);
Aa=zeros(12,6*nc); Aa(1:6,1:6)=(M0\M)./m.ddqmax; Aa(7:12,1:6)=(Me\M)./m.ddqmax;
ab=[(M0\(b-b0))./m.ddqmax;(Me\(b-be))./m.ddqmax];
mapping_s=toc(mapping); assembly=tic;
unforced=p.O{k}*x+p.offset{k}; target=reshape(p.tr.q(:,k+1:k+np),[],1);
Lq=p.L{k}(p.ixq,:); err=unforced(p.ixq)-target;
sw=c.smooth_weight*c.smooth_multiplier;
H=2*(p.trackH{k}+kron(c.effort_weight*p.holdgram+sw*p.diffgram,N'*N));
gs=zeros(6*nc,1); gs(1:6)=N'*(bb-previous./m.taumax);
g=2*(Lq'*err/c.qscale^2+c.effort_weight*kron(p.holdcount,N'*bb)+sw*gs);
H=(H+H')/2+1e-10*eye(6*nc);
ac=p.accO{k}*x+p.accC{k}; a0=[unforced(p.ixq);unforced(p.ixv);ac];
As=p.stateA{k}; as=a0./p.limits;
A=[U;-U;As;-As;Aa;-Aa]; rhs=[1-ub;1+ub;1-as;1+as;1-ab;1+ab];
ds=1./sqrt(diag(H)); Hw=H.*(ds*ds'); Aw=A.*ds'; gw=g.*ds;
assembly_s=toc(assembly); st=tic; y=-(Hw\gw);
if max(Aw*y-rhs)<=c.qp_tol
    flag=1; out=struct('iterations',0); cost=.5*y'*Hw*y+gw'*y;
else
    [y,cost,flag,out]=quadprog(Hw,gw,Aw,rhs,[],[],[],[],p.warm./ds,p.options);
end
if isempty(y), z=[]; else, z=ds.*y; end
solver_s=toc(st); fallback=flag<=0 || isempty(z) || any(~isfinite(z));
if ~fallback, residual=max([0;A*z-rhs]); fallback=residual>1e-5; else, residual=NaN; end
if fallback, desired=b0+M0*(-2*x(7:12)); else, desired=M*z(1:6)+b; end
raw=desired./ahat; u=max(-m.taumax,min(m.taumax,raw));
if isempty(cost), cost=NaN; end
log=[flag,fallback,residual,out.iterations,cost,mapping_s,assembly_s,solver_s,toc(total)];
d=struct('tau_desired',desired,'tau_raw',raw,'decision',z,'M',M,'bias',b);
if isfield(p,'audit_qp') && p.audit_qp
    d.H=H; d.g=g; d.A=A; d.rhs=rhs; d.U=U; d.ub=ub;
    d.DU=p.D*U; d.db=p.D*ub; d.db(1:6)=d.db(1:6)-previous./m.taumax;
end
end
