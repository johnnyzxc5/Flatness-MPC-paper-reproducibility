function result=simulate(p,plant,nom,c,fault,obs,seed)
%SIMULATE Same plant, clock, random streams and metrics as immutable v1.
twall=tic; t=0:c.h:c.Tf; nt=numel(t); nsub=round(c.Ts/c.h); nk=round(c.Tf/c.Ts);
assert(abs(nsub*c.h-c.Ts)<1e-12);
stream=RandStream('mt19937ar','Seed',seed+100000);
noiseq=obs.sigma_q*randn(stream,6,nk); noisedq=obs.sigma_dq*randn(stream,6,nk);
streamA=RandStream('mt19937ar','Seed',seed+200000); rho=exp(-c.Ts/.1);
na=zeros(1,nk); na(1)=obs.sigma_alpha*randn(streamA);
for k=2:nk, na(k)=rho*na(k-1)+obs.sigma_alpha*sqrt(1-rho^2)*randn(streamA); end
tc=(0:nk-1)*c.Ts; ah=ftmpc.alpha(tc-obs.delay_ticks*c.Ts,fault);
ah(2,:)=max(c.alpha_floor,min(1,ah(2,:)+obs.bias+na));
raw=struct('t',t,'tc',tc,'x',zeros(12,nt),'ddq',zeros(6,nt),'tau_applied',zeros(6,nt), ...
    'tau_cmd',zeros(6,nk),'tau_raw',zeros(6,nk),'tau_desired',zeros(6,nk), ...
    'measured',zeros(12,nk),'alpha_hat',ah,'alpha_true',ftmpc.alpha(t,fault), ...
    'noise_q',noiseq,'noise_dq',noisedq,'noise_alpha',na,'solver',zeros(nk,9), ...
    'decision',NaN(6*c.Nc,nk),'mapping_M',zeros(6,6,nk),'mapping_bias',zeros(6,nk), ...
    'anchors',NaN(12,c.Np,nk),'stage_maps',NaN(6,6,c.Np,nk),'stage_biases',NaN(6,c.Np,nk));
raw.solver_columns={'exitflag','fallback','primal_residual','iterations','cost','mapping_s','assembly_s','QP_s','controller_s'};
x=[p.tr.q(:,1);p.tr.dq(:,1)]; raw.x(:,1)=x;
[~,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),nom); previous=max(-nom.taumax,min(nom.taumax,b));
failure=''; kdone=0; p.has_warm=false; p.warm(:)=0;
for k=1:nk
    measured=x+[noiseq(:,k);noisedq(:,k)];
    [u,log,d]=ftv3.step(p,k,measured,ah(:,k),previous,nom);
    if ~isempty(d.decision), raw.decision(:,k)=d.decision; end
    raw.mapping_M(:,:,k)=d.M; raw.mapping_bias(:,k)=d.bias;
    if isfield(d,'anchors')
        raw.anchors(:,:,k)=d.anchors; raw.stage_maps(:,:,:,k)=d.maps; raw.stage_biases(:,:,k)=d.biases;
    end
    p.has_warm=~isempty(d.decision) && log(2)==0;
    if p.has_warm, p.warm=[d.decision(7:end);d.decision(end-5:end)]; else, p.warm(:)=0; end
    [xs,as,ta]=ftmpc_integrate_mex(x,u,tc(k),c.h,nsub,plant,fault(:));
    ix=(k-1)*nsub+(1:nsub+1); raw.x(:,ix)=xs; raw.ddq(:,ix)=as; raw.tau_applied(:,ix)=ta;
    raw.tau_cmd(:,k)=u; raw.tau_raw(:,k)=d.tau_raw; raw.tau_desired(:,k)=d.tau_desired;
    raw.measured(:,k)=measured; raw.solver(k,:)=log; x=xs(:,end); previous=u; kdone=k;
    if any(~isfinite(x)) || any(abs(x(1:6))>4*pi) || any(abs(x(7:12))>20)
        failure='Numerical/domain guard (not a certified safety stop)'; break;
    end
end
result=struct('raw',raw,'complete',isempty(failure),'failure',failure,'ticks_completed',kdone, ...
    'config',c,'fault',fault,'observation',obs,'seed',seed,'controller',p.variant, ...
    'plant',plant,'nominal_model',nom,'reference',p.tr,'prep_seconds',p.prep_seconds,'wall_seconds',toc(twall));
result.metrics=ftmpc.metrics(result);
end

