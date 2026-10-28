function f=faults(tr)
%FAULTS Demand-based scenarios chosen without controller performance.
% [start,end,final health]; demands refer to exact nominal reference only.
t=tr.t; demand=tr.alpha_required(2,:);
start=.1; stop=2.4; r=max(0,min(1,(t-start)/(stop-start))); good=r>0;
critical=max(1-(1-demand(good))./r(good));
mild=min(1,max(.15,critical+.04)); moderate=max(.10,critical-.03);
f=[10 11 1;start stop mild;start stop moderate;.1 1 .10];
end
