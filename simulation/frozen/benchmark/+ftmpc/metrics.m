function a=metrics(r)
raw=r.raw; m=r.nominal_model; c=r.config; n=r.ticks_completed;
last=round(n*c.Ts/c.h)+1; t=raw.t(1:last); T=t(end); x=raw.x(:,1:last);
[qr,~,~]=ftmpc.reference(t,r.reference,c.Tf); e=x(1:6,:)-qr;
pe=zeros(3,last);
for j=1:last, pe(:,j)=ftmpc.fk(x(1:6,j),m)-ftmpc.fk(qr(:,j),m); end
u=raw.tau_cmd(:,1:n); du=diff(u,1,2); sol=raw.solver(1:n,:);
ea=ftmpc.effort_applied(u,raw.tc(1:n),c.Ts,r.fault);
a=struct('complete',r.complete,'duration_s',T,'rmse_q',sqrt(trapz(t,sum(e.^2,1))/(6*T)), ...
    'rmse_cart_mm',1000*sqrt(trapz(t,sum(pe.^2,1))/T),'max_cart_mm',1000*max(vecnorm(pe)), ...
    'effort_cmd',sum(u.^2,'all')*c.Ts,'effort_applied',sum(ea), ...
    'TV_cmd',sum(abs(du),'all'),'rate_RMS_cmd',sqrt(sum(du.^2,'all')/(6*max(1,n-1)*c.Ts^2)), ...
    'fallback_fraction',mean(sol(:,2)),'saturation_fraction',mean(any(abs(u)>=m.taumax-1e-7,1)), ...
    'max_q_violation',max([0;reshape(abs(x(1:6,:))-m.qmax,[],1)]), ...
    'max_dq_violation',max([0;reshape(abs(x(7:12,:))-m.dqmax,[],1)]), ...
    'max_ddq_violation',max([0;reshape(abs(raw.ddq(:,1:last))-m.ddqmax,[],1)]), ...
    'max_cmd_violation',max([0;reshape(abs(u)-m.taumax,[],1)]), ...
    'controller_median_ms',1000*median(sol(:,9)),'controller_p95_ms',1000*prctile(sol(:,9),95), ...
    'controller_max_ms',1000*max(sol(:,9)),'deadline_miss_fraction',mean(sol(:,9)>c.Ts), ...
    'mapping_median_ms',1000*median(sol(:,6)),'QP_median_ms',1000*median(sol(:,8)), ...
    'prep_seconds',r.prep_seconds,'wall_seconds',r.wall_seconds);
% Explicit per-joint metrics and positive violation occupancy.
a.rmse_q_per_joint=sqrt(trapz(t,e.^2,2)/T)';
a.TV_per_joint=sum(abs(du),2)';
a.effort_applied_per_joint=ea';
a.q_violation_fraction=mean(any(abs(x(1:6,:))>m.qmax+1e-7,1));
a.dq_violation_fraction=mean(any(abs(x(7:12,:))>m.dqmax+1e-7,1));
a.ddq_violation_fraction=mean(any(abs(raw.ddq(:,1:last))>m.ddqmax+1e-7,1));
a.QP_iteration_limit_fraction=mean(sol(:,1)==0);
a.QP_infeasible_fraction=mean(sol(:,1)==-2);
a.torque_bound_violation=max([0;reshape(abs(raw.tau_applied(:,1:last))-raw.alpha_true(:,1:last).*m.taumax,[],1)]);
mask=t>=r.fault(1);
if nnz(mask)>1
    tf=t(mask); ef=e(:,mask); pf=pe(:,mask); dur=tf(end)-tf(1);
    a.postfault_rmse_q=sqrt(trapz(tf,sum(ef.^2,1))/(6*dur));
    a.postfault_rmse_cart_mm=1000*sqrt(trapz(tf,sum(pf.^2,1))/dur);
else
    a.postfault_rmse_q=NaN; a.postfault_rmse_cart_mm=NaN;
end
end
