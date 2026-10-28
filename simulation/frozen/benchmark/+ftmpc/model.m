function m = model(perturb)
%MODEL Six-axis Gen3 parameters from the pinned manufacturer URDF.
% Coordinates are URDF joint coordinates. Reflected rotor inertia and smooth
% friction are explicit study assumptions inherited from the previous code.
if nargin<1, perturb=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
doc=xmlread(fullfile(root,'third_party','kinova_gen3_6dof.urdf'));
links=doc.getElementsByTagName('link'); joints=doc.getElementsByTagName('joint');
m.T=zeros(4,4,7); m.X=zeros(6,6,6); m.G=zeros(6,6,6);
m.mass=zeros(6,1); m.com=zeros(3,6); m.Ic=zeros(3,3,6);
m.qmax=[pi;2.24;2.57;pi;2.09;pi]; % continuous joints: study workspace bounds
m.dqmax=[1.3963;1.3963;1.3963;1.2218;1.2218;1.2218];
m.taumax=[39;39;39;9;9;9];
m.ddqmax=[5.2;5.2;5.2;10;10;10]; % study bounds, not certified manufacturer limits
m.rotor=[.172;.172;.172;.089;.089;.089];
m.fv=[2;2;2;1.36;1.36;1.36]; m.fc=[2.15;2.15;2.15;1.6;1.6;1.6];
dm=getfield_default(perturb,'mass',zeros(6,1));
dc=getfield_default(perturb,'com',zeros(3,6));
di=getfield_default(perturb,'inertia',zeros(6,1));
m.fv=m.fv.*(1+getfield_default(perturb,'fv',zeros(6,1)));
m.fc=m.fc.*(1+getfield_default(perturb,'fc',zeros(6,1)));
for i=1:7
    j=joints.item(i-1); o=j.getElementsByTagName('origin').item(0);
    p=sscanf(char(o.getAttribute('xyz')),'%f'); a=sscanf(char(o.getAttribute('rpy')),'%f');
    R=eul2rotm(a([3 2 1])','ZYX'); T=[R,p;0 0 0 1]; m.T(:,:,i)=T;
    if i==7, continue; end
    Ti=[R',-R'*p;0 0 0 1]; m.X(:,:,i)=adj(Ti);
    child=char(j.getElementsByTagName('child').item(0).getAttribute('link'));
    for k=0:links.getLength-1
        l=links.item(k); if strcmp(char(l.getAttribute('name')),child), break; end
    end
    in=l.getElementsByTagName('inertial').item(0);
    mass=str2double(char(in.getElementsByTagName('mass').item(0).getAttribute('value')))*(1+dm(i));
    c=sscanf(char(in.getElementsByTagName('origin').item(0).getAttribute('xyz')),'%f')+dc(:,i);
    n=in.getElementsByTagName('inertia').item(0);
    v=zeros(6,1); names={'ixx','iyy','izz','iyz','ixz','ixy'};
    for k=1:6, v(k)=str2double(char(n.getAttribute(names{k}))); end
    I=[v(1),v(6),v(5);v(6),v(2),v(4);v(5),v(4),v(3)]*(1+di(i));
    ev=eig(I); assert(mass>0 && min(ev)>0 && 2*max(ev)<sum(ev),'Nonphysical inertia');
    C=skew(c); m.G(:,:,i)=[I-mass*C*C,mass*C;-mass*C,mass*eye(3)];
    m.mass(i)=mass; m.com(:,i)=c; m.Ic(:,:,i)=I;
end
end
function x=getfield_default(s,k,d)
if isfield(s,k), x=s.(k); else, x=d; end
end
function C=skew(v)
C=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end
function A=adj(T)
R=T(1:3,1:3); A=[R,zeros(3);skew(T(1:3,4))*R,R];
end
