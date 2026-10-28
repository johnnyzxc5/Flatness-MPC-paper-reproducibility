function r=simulate(kind,tr,Tnom,fault,m,plant,c,o,obs,seed,xoffset)
treatment='constant'; v=ftv6.variant(kind);
wall=tic; path=ftv5.path_cache(tr,m,o); p=ftv5.template(path,m,c);
plan=ftv5.phase_plan(0,0,0,Tnom); plan_start=0; history=[];
nk=round(o.max_time/c.Ts); ns=round(c.Ts/c.h); nt=nk*ns+1;
raw=struct('t',(0:nt-1)*c.h,'tc',(0:nk-1)*c.Ts, ...
    'x',NaN(12,nk+1),'fine_x',NaN(12,nt),'fine_ddq',NaN(6,nt),'fine_tau',NaN(6,nt), ...
    'qref',NaN(6,nk),'phase',NaN(3,nk),'u',NaN(6,nk),'health',NaN(6,nk), ...
    'slope',NaN(6,nk),'predicted_health',NaN(6,c.Np,nk),'predictor_mode',NaN(6,nk), ...
    'solver',NaN(nk,9),'predictor_s',zeros(1,nk),'governor_s',zeros(1,nk), ...
    'reference_s',zeros(1,nk),'full_tick_s',zeros(1,nk),'violation',zeros(3,nk),'linearizations',zeros(1,nk), ...
    'observer_s',zeros(1,nk),'disturbance',zeros(6,nk),'observer_updated',false(1,nk),'state_active',false(1,nk));
stream=RandStream('mt19937ar','Seed',seed+100000);
noise=[obs.sigma_q*randn(stream,6,nk);obs.sigma_dq*randn(stream,6,nk)]; raw.noise=noise; raw.measured=NaN(12,nk);
x=[tr.q0;zeros(6,1)]+xoffset; raw.x(:,1)=x; raw.fine_x(:,1)=x;
prediction=[]; prior_x=x; prior_health=ones(6,1); p.disturbance=zeros(6,1);
[~,previous]=ftmpc_dynamics_mex(x(1:6),x(7:12),m);
previous=max(-m.taumax,min(m.taumax,previous)); events={}; stop='time_limit';
settled_since=NaN; count=0; replan_count=0; exhausted=0;
for k=1:nk
    tick=tic; t=raw.tc(k); health=ftmpc.alpha(t,fault);
    measured=x+noise(:,k); raw.measured(:,k)=measured;
    if any(health<path.floor-1e-10)
        stop='outside_operating_envelope_no_brake_model'; break;
    end
    ot=tic;
    if v.observer
        [p.disturbance,oi]=ftv6.update_observer(p.disturbance,measured,prediction,prior_x,health,prior_health,m,c,v);
        raw.observer_updated(k)=oi.updated;
    end
    raw.disturbance(:,k)=p.disturbance; raw.observer_s(k)=toc(ot);
    pt=tic; [history,estimate]=ftv5.predict(history,health,c.Ts,o,path.floor);
    raw.predictor_s(k)=toc(pt); raw.slope(:,k)=estimate.slope; raw.predictor_mode(:,k)=estimate.mode;
    used=estimate; if ~ismember(treatment,{'trend','trend_governor'}), used.slope(:)=0; end
    age=t-plan_start;
    if ~strcmp(treatment,'fixed') && mod(k-1,round(o.governor_period/c.Ts))==0
        [candidate,info]=ftv5.govern(plan,age,path,m,used,o,Tnom);
        raw.governor_s(k)=info.seconds;
        if ~strcmp(info.status,'keep')
            e=struct('time',t,'info',info,'estimate',estimate,'used_estimate',used, ...
                'old_plan',plan,'old_age',age,'candidate',candidate);
            events{end+1}=e; %#ok<AGROW>
        end
        if ~isempty(candidate)
            plan=candidate; plan_start=t; age=0; replan_count=replan_count+1;
        elseif strcmp(info.status,'search_exhausted')
            % Search failure is not a proof of infeasibility. Retain the prior
            % reference and report the event; do not invent a successful stop.
            exhausted=exhausted+1;
        end
    end
    rt=tic; ref=ftv5.local_reference(plan,age,path,m,c); raw.reference_s(k)=toc(rt);
    controller_estimate=used;
    if strcmp(treatment,'trend_governor'), controller_estimate.slope(:)=0; end
    health_preview=ftv5.forecast(controller_estimate,(0:c.Np-1)*c.Ts);
    [u,log,d,cache]=ftv6.step(p,kind,ref,measured,health_preview,previous,m);
    p.cache=cache; raw.linearizations(k)=cache.evaluations;
    raw.state_active(k)=cache.state_active;
    if ~isempty(d.decision) && log(2)==0, p.warm=[d.decision(7:end);d.decision(end-5:end)]; else, p.warm(:)=0; end
    if v.observer
        ot=tic;
        nominal=ftmpc_integrate_mex(measured,health.*u,0,v.internal_h,round(c.Ts/v.internal_h),m,[10;11;1]);
        prediction=nominal(:,end); prior_x=measured; prior_health=health;
        raw.observer_s(k)=raw.observer_s(k)+toc(ot);
    end
    raw.full_tick_s(k)=toc(tick);
    [xs,acc,ta]=ftmpc_integrate_mex(x,u,t,c.h,ns,plant,fault(:));
    ix=(k-1)*ns+(1:ns+1); raw.fine_x(:,ix)=xs; raw.fine_ddq(:,ix)=acc; raw.fine_tau(:,ix)=ta;
    raw.x(:,k+1)=xs(:,end); raw.qref(:,k)=ref.q(:,1);
    [ss,vv,aa]=ftv5.phase(plan,age); raw.phase(:,k)=[ss;vv;aa];
    raw.u(:,k)=u; raw.health(:,k)=health; raw.predicted_health(:,:,k)=health_preview; raw.solver(k,:)=log;
    raw.violation(:,k)=[max([0;max(abs(xs(1:6,:))-m.qmax,[],2)]); ...
        max([0;max(abs(xs(7:12,:))-m.dqmax,[],2)]);max([0;max(abs(acc)-m.ddqmax,[],2)])];
    x=xs(:,end); previous=u; count=k;
    if any(~isfinite(x)) || any(abs(x(1:6))>4*pi) || any(abs(x(7:12))>20)
        stop='numerical_or_domain_guard'; break;
    end
    settled=age+c.Ts>=plan.T && max(abs(x(1:6)-tr.q1))<.02 && max(abs(x(7:12)))<.02;
    if settled
        if isnan(settled_since), settled_since=t; end
        if t-settled_since>=o.hold_time, stop='goal_held'; end
    else, settled_since=NaN; stop='time_limit'; end
end
n=count; assert(n>0,'No in-scope control steps recorded');
err=raw.x(1:6,1:n)-raw.qref(:,1:n);
dpath=path.d; projection=(dpath'*(raw.x(1:6,1:n+1)-path.q0))/(dpath'*dpath);
transverse=raw.x(1:6,1:n+1)-(path.q0+dpath*projection);
refprojection=(dpath'*(raw.qref(:,1:n)-path.q0))/(dpath'*dpath);
reftransverse=raw.qref(:,1:n)-(path.q0+dpath*refprojection);
metrics=struct('stop',stop,'goal_reached',strcmp(stop,'goal_held'),'duration_s',n*c.Ts, ...
    'rmse_adapted_rad',sqrt(mean(err.^2,'all')),'max_path_error_rad',max(vecnorm(transverse)), ...
    'reference_path_error_rad',max(vecnorm(reftransverse)),'goal_error_rad',max(abs(x(1:6)-tr.q1)), ...
    'speed_final',max(abs(x(7:12))),'max_q_violation',max(raw.violation(1,1:n)), ...
    'max_dq_violation',max(raw.violation(2,1:n)),'max_ddq_violation',max(raw.violation(3,1:n)), ...
    'fallback_fraction',mean(raw.solver(1:n,2)),'replans',replan_count,'search_exhausted',exhausted, ...
    'TV_cmd',sum(abs(diff(raw.u(:,1:n),1,2)),'all'),'effort_cmd',c.Ts*sum(raw.u(:,1:n).^2,'all'), ...
    'full_tick_p95_ms',1e3*prctile(raw.full_tick_s(1:n),95),'full_tick_max_ms',1e3*max(raw.full_tick_s(1:n)), ...
    'deadline_exceedances',sum(raw.full_tick_s(1:n)>c.Ts),'governor_max_ms',1e3*max(raw.governor_s(1:n)), ...
    'model_p95_ms',1e3*prctile(raw.solver(1:n,6),95),'linearizations_total',sum(raw.linearizations(1:n)), ...
    'state_map_fraction',mean(raw.state_active(1:n)),'observer_p95_ms',1e3*prctile(raw.observer_s(1:n),95), ...
    'max_disturbance_fraction',max(abs(raw.disturbance(:,1:n))./m.taumax,[],'all'));
moving=raw.tc(1:n)<Tnom; tail=raw.tc(1:n)>=o.max_time-1;
metrics.rmse_motion_rad=sqrt(mean(err(:,moving).^2,'all'));
metrics.rmse_terminal_rad=sqrt(mean((raw.x(1:6,find(tail))-tr.q1).^2,'all'));
metrics.goal_reached=n==nk && all(abs(raw.x(1:6,find(tail))-tr.q1)<.02,'all') && all(abs(raw.x(7:12,find(tail)))<.02,'all');
if metrics.goal_reached, metrics.stop='goal_held'; end
r=struct('controller',kind,'treatment',treatment,'raw',raw,'events',{events},'metrics',metrics, ...
    'fault',fault,'config',c,'options',o,'path',path,'nominal_model',m,'ticks',n,'wall_s',toc(wall), ...
    'original_reference',tr,'Tnom',Tnom,'plant',plant,'observation',obs,'seed',seed,'variant',v,'xoffset',xoffset);
end
