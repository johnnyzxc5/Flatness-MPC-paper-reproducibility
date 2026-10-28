function [q,dq,ddq]=reference(t,tr,Tf)
s=max(0,min(1,t(:)'/Tf)); d=tr.q1-tr.q0;
q=tr.q0+d*(10*s.^3-15*s.^4+6*s.^5);
dq=d*((30*s.^2-60*s.^3+30*s.^4)/Tf);
ddq=d*((60*s-180*s.^2+120*s.^3)/Tf^2);
end
