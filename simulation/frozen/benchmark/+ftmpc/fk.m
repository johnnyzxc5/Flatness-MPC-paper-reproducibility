function [p,U] = fk(q,m)
%FK Direct URDF chain, independent from spatial dynamics construction.
T=eye(4); U=0;
for i=1:6
    c=cos(q(i)); s=sin(q(i)); R=[c -s 0;s c 0;0 0 1];
    T=T*m.T(:,:,i)*[R zeros(3,1);0 0 0 1];
    pc=T*[m.com(:,i);1]; U=U+9.81*m.mass(i)*pc(3);
end
T=T*m.T(:,:,7); p=T(1:3,4);
end
