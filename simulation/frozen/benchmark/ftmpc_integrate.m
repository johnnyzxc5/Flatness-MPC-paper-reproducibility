function [xs,as,taus] = ftmpc_integrate(x,u,t0,h,nstep,m,fault)
%FTMPC_INTEGRATE RK4 with held drive command; physical health may vary inside tick.
%#codegen
xs=zeros(12,nstep+1); as=zeros(6,nstep+1); taus=zeros(6,nstep+1); xs(:,1)=x;
for j=1:nstep
    t=t0+(j-1)*h;
    [k1,ta]=rhs(x,u,t,m,fault); k2=rhs(x+h*k1/2,u,t+h/2,m,fault);
    k3=rhs(x+h*k2/2,u,t+h/2,m,fault); k4=rhs(x+h*k3,u,t+h,m,fault);
    as(:,j)=k1(7:12); taus(:,j)=ta;
    x=x+h*(k1+2*k2+2*k3+k4)/6; xs(:,j+1)=x;
end
[dx,ta]=rhs(x,u,t0+nstep*h,m,fault); as(:,end)=dx(7:12); taus(:,end)=ta;
end
function [dx,ta]=rhs(x,u,t,m,f)
a=ones(6,1); z=max(0,min(1,(t-f(1))/max(f(2)-f(1),eps)));
if t>=f(1), a(2)=1-(1-f(3))*z; end
ta=a.*u; [M,b]=ftmpc_dynamics(x(1:6),x(7:12),m);
dx=[x(7:12);M\(ta-b)];
end
