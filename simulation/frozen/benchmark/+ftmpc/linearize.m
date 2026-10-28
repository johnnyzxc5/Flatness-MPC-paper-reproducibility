function [Ad,Bd,d,Ac,Bc,cc]=linearize(x,u,m,Ts)
%LINEARIZE Absolute affine model; exact ZOH of the local affine dynamics.
[M,b]=ftmpc_dynamics_mex(x(1:6),x(7:12),m); a=M\(u-b);
Ac=[zeros(6) eye(6);zeros(6,12)]; Bc=[zeros(6);M\eye(6)];
epsx=1e-5;
for j=1:12
    xp=x; xm=x; xp(j)=xp(j)+epsx; xm(j)=xm(j)-epsx;
    [Mp,bp]=ftmpc_dynamics_mex(xp(1:6),xp(7:12),m);
    [Mm,bm]=ftmpc_dynamics_mex(xm(1:6),xm(7:12),m);
    Ac(7:12,j)=((Mp\(u-bp))-(Mm\(u-bm)))/(2*epsx);
end
cc=[x(7:12);a]-Ac*x-Bc*u;
E=expm([Ac Bc cc;zeros(7,19)]*Ts);
Ad=E(1:12,1:12); Bd=E(1:12,13:18); d=E(1:12,19);
end
