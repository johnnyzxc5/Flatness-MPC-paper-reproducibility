function [u,log,detail]=step(p,k,x,ahat,previous,m)
%STEP Convex QP with comparable physical command effort and variation costs.
alltime=tic; c=p.c; np=c.Np; nc=c.Nc; ahat=max(c.alpha_floor,min(1,ahat));
mapping=tic;
if strcmp(p.kind,'Flatness')
    [M,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
    % Only one 6x6 M and one bias evaluation per tick. No scalar envelopes,
    % inverse mass, or vertex enumeration are needed for the coupled polytope.
else
    M=eye(6); b=zeros(6,1);
end
N=M./(ahat.*m.taumax); bb=b./(ahat.*m.taumax);
U=kron(p.hold,N); ub=repmat(bb,np,1);
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
% Normalize inequality rows by physical scales identically across methods.
A=[U;-U;As;-As]; rhs=[ones(6*np,1)-ub;ones(6*np,1)+ub;ones(size(as))-as;ones(size(as))+as];
% Common diagonal variable scaling retains the original sparsity pattern.
% Test the exact unconstrained minimizer first for EVERY controller.
ds=1./sqrt(diag(H)); Hw=H.*(ds*ds'); Aw=A.*ds'; gw=g.*ds;
assembly_s=toc(assembly); solvertime=tic;
y=-(Hw\gw);
if max(Aw*y-rhs)<=c.qp_tol
    flag=1; out=struct('iterations',0); cost=.5*y'*Hw*y+gw'*y;
else
    y0=p.warm./ds;
    [y,cost,flag,out]=quadprog(Hw,gw,Aw,rhs,[],[],[],[],y0,p.options);
end
if isempty(y), z=[]; else, z=ds.*y; end
solver_s=toc(solvertime); fallback=flag<=0 || isempty(z) || any(~isfinite(z));
if ~fallback
    residual=max([0;A*z-rhs]); fallback=residual>1e-5;
else, residual=NaN; end
if fallback
    % Common bounded damping/gravity fallback; no claims of state safety.
    [Mf,bf]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
    desired=bf+Mf*(-2*x(7:12)); raw=desired./ahat;
else
    if strcmp(p.kind,'Flatness'), desired=M*z(1:6)+b; else, desired=z(1:6); end
    raw=desired./ahat;
end
u=max(-m.taumax,min(m.taumax,raw));
iters=out.iterations; if isempty(cost), cost=NaN; end
log=[flag,fallback,residual,iters,cost,mapping_s,assembly_s,solver_s,toc(alltime)];
detail=struct('tau_desired',desired,'tau_raw',raw,'saturated',abs(raw-u)>1e-9, ...
    'decision',z,'M',M,'bias',b);
if isfield(p,'audit_qp') && p.audit_qp
    detail.H=H; detail.g=g; detail.A=A; detail.rhs=rhs;
end
end
