function [u,log,d,cache]=step(p,kind,r,x,health,previous,m)
%STEP Local horizon only, with stage-dependent physical command limits/cost.
total=tic; c=p.c; np=c.Np; nc=c.Nc; modeltime=tic;
health=max(c.alpha_floor,min(1,health));
if strcmp(kind,'LTV'), stages=1:np; else, stages=1; end
keys=[r.q(:,stages);r.dq(:,stages);r.ddq(:,stages)];
cache=struct('keys',keys,'Ad',zeros(12,12,numel(stages)),'Bd',zeros(12,6,numel(stages)), ...
    'Ac',zeros(12,12,numel(stages)),'Bc',zeros(12,6,numel(stages)),'evaluations',0);
for j=stages
    hit=[];
    if isfield(p,'cache'), hit=find(max(abs(p.cache.keys-keys(:,j)),[],1)<1e-11,1); end
    if ~isempty(hit)
        cache.Ad(:,:,j)=p.cache.Ad(:,:,hit); cache.Bd(:,:,j)=p.cache.Bd(:,:,hit);
        cache.Ac(:,:,j)=p.cache.Ac(:,:,hit); cache.Bc(:,:,j)=p.cache.Bc(:,:,hit);
    else
        [cache.Ad(:,:,j),cache.Bd(:,:,j),~,cache.Ac(:,:,j),cache.Bc(:,:,j)]= ...
            ftmpc.linearize([r.q(:,j);r.dq(:,j)],r.tau(:,j),m,c.Ts);
        cache.evaluations=cache.evaluations+1;
    end
end
if strcmp(kind,'LTV')
    Ad=cache.Ad; Bd=cache.Bd; ds=zeros(12,np);
    Ac=cache.Ac; Bc=cache.Bc; cc=zeros(12,np);
    for j=1:np
        xr=[r.q(:,j);r.dq(:,j)]; xn=[r.q(:,j+1);r.dq(:,j+1)];
        ds(:,j)=xn-Ad(:,:,j)*xr-Bd(:,:,j)*r.tau(:,j);
        cc(:,j)=[r.dq(:,j);r.ddq(:,j)]-Ac(:,:,j)*xr-Bc(:,:,j)*r.tau(:,j);
    end
    [O,L,offset]=ftmpc.lift(Ad,Bd,ds,np,nc);
    ao=zeros(6*np,12); al=zeros(6*np,6*nc); ac=zeros(6*np,1);
    for j=1:np
        ix=(j-1)*6+(1:6);
        if j==1, ox=eye(12); lu=zeros(12,6*nc); cx=zeros(12,1);
        else, prev=(j-2)*12+(1:12); ox=O(prev,:); lu=L(prev,:); cx=offset(prev); end
        ax=Ac(7:12,:,j); bu=Bc(7:12,:,j);
        ao(ix,:)=ax*ox; al(ix,:)=ax*lu+bu*p.E(ix,:); ac(ix)=ax*cx+cc(7:12,j);
    end
    M=eye(6); b=zeros(6,1); Aa=zeros(0,6*nc); ab=zeros(0,1);
else
    O=p.O{1}; L=p.L{1}; offset=p.offset{1}; ao=p.accO{1}; al=p.accL{1}; ac=p.accC{1};
    Bd=cache.Bd(:,:,1);
    M=(c.Ts*eye(6))/Bd(7:12,:);
    [M0,b0]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
    ta=M0*r.ddq(:,1)+b0;
    xs=ftmpc_integrate_mex(x,ta,0,.001,round(c.Ts/.001),m,[10;11;1]); xe=xs(:,end);
    b=ta-M*(xe(7:12)-x(7:12))/c.Ts;
    [Me,be]=ftmpc_dynamics_mex(xe(1:6),xe(7:12),m);
    Aa=zeros(12,6*nc); Aa(1:6,1:6)=(M0\M)./m.ddqmax; Aa(7:12,1:6)=(Me\M)./m.ddqmax;
    ab=[(M0\(b-b0))./m.ddqmax;(Me\(b-be))./m.ddqmax];
end
model_s=toc(modeltime); assembly=tic;
scale=reshape(health.*m.taumax,[],1);
U=kron(p.hold,M)./scale; ub=repmat(b,np,1)./scale;
DU=p.D*U; db=p.D*ub; db(1:6)=db(1:6)-previous./m.taumax;
unforced=O*x+offset; target=reshape(r.q(:,2:np+1),[],1);
Lq=L(p.ixq,:); err=unforced(p.ixq)-target; sw=c.smooth_weight*c.smooth_multiplier;
H=2*(Lq'*Lq/c.qscale^2+c.effort_weight*(U'*U)+sw*(DU'*DU));
g=2*(Lq'*err/c.qscale^2+c.effort_weight*U'*ub+sw*DU'*db);
H=(H+H')/2+1e-10*eye(6*nc);
as=[unforced(p.ixq);unforced(p.ixv);ao*x+ac]./p.limits;
As=[Lq;L(p.ixv,:);al]./p.limits;
A=[U;-U;As;-As;Aa;-Aa]; rhs=[1-ub;1+ub;1-as;1+as;1-ab;1+ab];
ds=1./sqrt(diag(H)); Hw=H.*(ds*ds'); Aw=A.*ds'; gw=g.*ds;
assembly_s=toc(assembly); solver=tic; y=-(Hw\gw);
if max(Aw*y-rhs)<=c.qp_tol
    flag=1; out=struct('iterations',0); cost=.5*y'*Hw*y+gw'*y;
else
    [y,cost,flag,out]=quadprog(Hw,gw,Aw,rhs,[],[],[],[],p.warm./ds,p.options);
end
if isempty(y), z=[]; else, z=ds.*y; end
solver_s=toc(solver); fallback=flag<=0 || isempty(z) || any(~isfinite(z));
if ~fallback, residual=max([0;A*z-rhs]); fallback=residual>1e-5; else, residual=NaN; end
if fallback
    [Mf,bf]=ftmpc_dynamics_mex(x(1:6),x(7:12),m); desired=bf-2*Mf*x(7:12);
else, desired=M*z(1:6)+b; end
u=max(-m.taumax,min(m.taumax,desired./health(:,1)));
if isempty(cost), cost=NaN; end
log=[flag fallback residual out.iterations cost model_s assembly_s solver_s toc(total)];
d=struct('decision',z,'H',H,'g',g,'A',A,'rhs',rhs,'U',U,'ub',ub,'DU',DU,'db',db);
end
