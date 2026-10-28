function [s,v,a]=phase(p,t)
z=max(0,min(1,t(:)'/p.T));
s=p.coef*[ones(size(z));z;z.^2;z.^3;z.^4;z.^5];
v=p.coef(2:6)*[ones(size(z));2*z;3*z.^2;4*z.^3;5*z.^4]/p.T;
a=p.coef(3:6)*[2*ones(size(z));6*z;12*z.^2;20*z.^3]/p.T^2;
ended=t(:)'>=p.T; s(ended)=1; v(ended)=0; a(ended)=0;
end
