function e=effort_applied(u,tc,Ts,fault)
%EFFORT_APPLIED Exact integral for ZOH drive commands and piecewise-linear health.
e=zeros(6,1);
for k=1:size(u,2)
    a=tc(k); b=a+Ts; cuts=sort(unique([a,b,fault(1:2)])); cuts=cuts(cuts>=a & cuts<=b);
    for j=1:numel(cuts)-1
        health=ftmpc.alpha(cuts(j:j+1),fault);
        weight=(cuts(j+1)-cuts(j))*(health(:,1).^2+health(:,1).*health(:,2)+health(:,2).^2)/3;
        e=e+u(:,k).^2.*weight;
    end
end
end
