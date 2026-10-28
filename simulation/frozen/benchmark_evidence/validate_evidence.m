function report=validate_evidence
report=validate_v6; m=ftmpc.model(); c=ftmpc.config(); c.smooth_multiplier=.001;
o=ftv5.options(); o.max_time=.2; tr=ftmpc.trajectory('Inertia','evaluation',m,c);
[~,obs]=ftmpc.perturbation('nominal',1); fault=[.04 .10 .65];
for kind={'Observer','LTVObserver'}
    a=ftv6.simulate(kind{1},tr,4.5,fault,m,m,c,o,obs,1,zeros(12,1));
    b=fte.simulate(kind{1},tr,4.5,fault,m,m,c,o,obs,1,zeros(12,1));
    assert(isequal(a.raw.x,b.raw.x) && isequal(a.raw.u,b.raw.u));
    assert(isequal(b.raw.health,b.raw.true_health));
end
obs.delay_ticks=2; b=fte.simulate('Observer',tr,4.5,fault,m,m,c,o,obs,1,zeros(12,1));
assert(max(abs(b.raw.health(:,3:end)-b.raw.true_health(:,1:end-2)),[],'all')<1e-14);
report.instrumentation_equivalence=true; report.causal_health_delay=true;
fprintf('Evidence wrapper exact equivalence and causal delay PASS\n');
end
