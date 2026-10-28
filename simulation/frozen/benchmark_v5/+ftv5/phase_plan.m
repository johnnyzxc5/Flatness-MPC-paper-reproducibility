function p=phase_plan(s,v,a,T)
c=[s,T*v,.5*T^2*a]; rhs=[1-s-c(2)-c(3),-c(2)-2*c(3),-2*c(3)];
p=struct('coef',[c,rhs/[1 3 6;1 4 12;1 5 20]],'T',T);
end
