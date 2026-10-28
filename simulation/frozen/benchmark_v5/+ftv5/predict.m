function [history,estimate]=predict(history,health,Ts,o,floor_health)
%PREDICT Causal exact-health pilot: local slope, jump reset, plateau detector.
if isempty(history), history=health(:); else, history=[history,health(:)]; end
history=history(:,max(1,end-o.window+1):end); n=size(history,2);
slope=zeros(6,1); mode=zeros(6,1); % 0 stable, -1 degrading, +1 recovering, 2 jump
for j=1:6
    y=history(j,:); delta=diff(y);
    if ~isempty(delta) && abs(delta(end))>=o.jump_threshold
        mode(j)=2; continue;
    end
    lastjump=find(abs(delta)>=o.jump_threshold,1,'last');
    if ~isempty(lastjump), y=y(lastjump+1:end); end
    if numel(y)>=o.plateau_samples && max(abs(diff(y(end-o.plateau_samples+1:end))))<1e-9
        continue;
    end
    if numel(y)>=2
        t=(0:numel(y)-1)*Ts; t=t-mean(t);
        slope(j)=sum(t.*(y-mean(y)))/sum(t.^2);
        if abs(slope(j))<o.slope_threshold, slope(j)=0; end
        mode(j)=sign(slope(j));
    end
end
estimate=struct('current',health(:),'slope',slope,'mode',mode, ...
    'floor',floor_health,'horizon',o.preview_seconds);
end
