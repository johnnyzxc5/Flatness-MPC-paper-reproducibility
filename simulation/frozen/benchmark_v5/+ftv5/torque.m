function [tau,A,b]=torque(path,s,v,a,m)
%TORQUE Straight path inverse dynamics with interpolated smooth coefficients.
s=max(0,min(1,s(:)')); v=v(:)'; a=a(:)';
z=s*(numel(path.s)-1); ix=min(floor(z)+1,numel(path.s)-1); w=z-(ix-1);
A=path.A(:,ix).*(1-w)+path.A(:,ix+1).*w;
C=path.C(:,ix).*(1-w)+path.C(:,ix+1).*w;
G=path.G(:,ix).*(1-w)+path.G(:,ix+1).*w;
velocity=path.d*v;
b=C.*v.^2+G+m.fv.*velocity+m.fc.*tanh(15*velocity);
tau=A.*a+b;
end
