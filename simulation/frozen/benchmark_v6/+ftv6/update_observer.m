function [d,info]=update_observer(d,x,prediction,previous_x,health,previous_health,m,c,v)
%UPDATE_OBSERVER Positive disturbance means missing torque to be compensated.
info=struct('updated',false,'raw',zeros(6,1));
if isempty(prediction) || max(abs(health-previous_health))>v.health_gate, return; end
[M,~]=ftmpc_dynamics_mex(.5*(x(1:6)+previous_x(1:6)),zeros(6,1),m);
residual=M*(prediction(7:12)-x(7:12))/c.Ts;
bound=v.observer_fraction*m.taumax; residual=max(-bound,min(bound,residual));
gain=1-exp(-c.Ts/v.observer_tc); d=max(-bound,min(bound,(1-gain)*d+gain*residual));
info.updated=true; info.raw=residual;
end
