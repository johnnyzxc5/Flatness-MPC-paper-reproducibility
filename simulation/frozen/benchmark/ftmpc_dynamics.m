function [M,b] = ftmpc_dynamics(q,dq,m)
%FTMPC_DYNAMICS Shared kinematics + CRBA mass + RNEA bias, six revolute joints.
%#codegen
% General strategy inspired by SHAPE shared-kinematics kernels. CRBA avoids
% six separate inverse-dynamics calls. Every controller uses this same kernel.
X=zeros(6,6,6); V=zeros(6,6,6); %#ok<NASGU>
vel=zeros(6,6); acc=zeros(6,6); force=zeros(6,6);
S=[0;0;1;0;0;0]; vp=zeros(6,1); ap=[0;0;0;0;0;9.81];
for i=1:6
    c=cos(q(i)); s=sin(q(i)); R=[c s 0;-s c 0;0 0 1];
    X(:,:,i)=blkdiag(R,R)*m.X(:,:,i);
    vi=X(:,:,i)*vp+S*dq(i); ai=X(:,:,i)*ap+motion_cross(vi)*(S*dq(i));
    vel(:,i)=vi; acc(:,i)=ai;
    force(:,i)=m.G(:,:,i)*ai-motion_cross(vi)'*(m.G(:,:,i)*vi);
    vp=vi; ap=ai;
end
M=zeros(6); b=zeros(6,1); Ic=m.G;
for i=6:-1:1
    b(i)=S'*force(:,i);
    F=Ic(:,:,i)*S; M(i,i)=S'*F;
    j=i;
    while j>1
        F=X(:,:,j)'*F; j=j-1; M(i,j)=S'*F; M(j,i)=M(i,j);
    end
    if i>1
        force(:,i-1)=force(:,i-1)+X(:,:,i)'*force(:,i);
        Ic(:,:,i-1)=Ic(:,:,i-1)+X(:,:,i)'*Ic(:,:,i)*X(:,:,i);
    end
end
M=M+diag(m.rotor);
b=b+m.fv.*dq+m.fc.*tanh(15*dq);
end
function A=motion_cross(v)
w=v(1:3); u=v(4:6);
W=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
U=[0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0];
A=[W zeros(3);U W];
end
