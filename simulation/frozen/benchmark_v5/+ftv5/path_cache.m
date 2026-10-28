function path=path_cache(tr,m,o)
tm=tic; s=linspace(0,1,o.grid); d=tr.q1-tr.q0;
A=zeros(6,numel(s)); C=A; G=A;
for k=1:numel(s)
    q=tr.q0+d*s(k); [M,g]=ftmpc_dynamics_mex(q,zeros(6,1),m);
    [~,b]=ftmpc_dynamics_mex(q,d,m);
    A(:,k)=M*d; G(:,k)=g; C(:,k)=b-g-m.fv.*d-m.fc.*tanh(15*d);
end
floor_health=.05*ones(6,1);
floor_health(2)=ceil(100*max(abs(G(2,:)))/m.taumax(2)/(1-o.static_reserve))/100;
path=struct('s',s,'q0',tr.q0,'q1',tr.q1,'d',d,'A',A,'C',C,'G',G, ...
    'floor',floor_health,'preparation_s',toc(tm));
end
