function [p,obs]=perturbation(mode,seed,value)
if nargin<3, value=0; end
p=struct('mass',zeros(6,1),'com',zeros(3,6),'inertia',zeros(6,1),'fv',zeros(6,1),'fc',zeros(6,1));
obs=struct('bias',0,'delay_ticks',0,'sigma_alpha',0,'sigma_q',0,'sigma_dq',0);
switch mode
    case 'historical'
        p.mass(:)=.05; p.com=repmat([.002;-.002;.002],1,6); p.inertia(:)=.03;
        p.fv=[.12;.05;.15;.08;.10;.04]; p.fc=[.08;.06;.10;.05;.09;.03];
    case 'mass'
        p.mass(:)=value;
    case 'random'
        stream=RandStream('mt19937ar','Seed',seed);
        p.mass=.1*(2*rand(stream,6,1)-1); p.com=.002*(2*rand(stream,3,6)-1);
        p.inertia=.05*(2*rand(stream,6,1)-1); p.fv=.15*(2*rand(stream,6,1)-1); p.fc=.1*(2*rand(stream,6,1)-1);
        obs.bias=.05*(2*rand(stream)-1); obs.delay_ticks=randi(stream,3)-1;
        obs.sigma_alpha=.01; obs.sigma_q=1e-4; obs.sigma_dq=1e-3;
end
end
