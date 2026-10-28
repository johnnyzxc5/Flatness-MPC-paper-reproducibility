function [O,L,c] = lift(A,B,d,Np,Nc)
%LIFT Exact affine state prediction. Hold last decision after Nc.
% A/B/d may be constant or genuinely time varying over the horizon.
nx=size(A,1); nu=size(B,2); O=zeros(nx*Np,nx); L=zeros(nx*Np,nu*Nc); c=zeros(nx*Np,1);
Ox=eye(nx); Lu=zeros(nx,nu*Nc); cx=zeros(nx,1);
for i=1:Np
    Ai=A(:,:,min(i,size(A,3))); Bi=B(:,:,min(i,size(B,3))); di=d(:,min(i,size(d,2)));
    Ox=Ai*Ox; Lu=Ai*Lu; cx=Ai*cx+di;
    col=(min(i,Nc)-1)*nu+(1:nu); Lu(:,col)=Lu(:,col)+Bi;
    ix=(i-1)*nx+(1:nx); O(ix,:)=Ox; L(ix,:)=Lu; c(ix)=cx;
end
end
