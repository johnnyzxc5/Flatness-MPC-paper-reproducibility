function inspect_v5
root=fileparts(mfilename('fullpath')); repo=fileparts(root);
addpath(root,fullfile(repo,'benchmark'),fullfile(repo,'benchmark','generated'));
m=ftmpc.model(); c=ftmpc.config(); o=ftv5.options();
for profile={'Inertia','Gravity'}
 tr=ftmpc.trajectory(profile{1},'evaluation',m,c);
 for reverse=[false true]
  if reverse, temp=tr.q0; tr.q0=tr.q1; tr.q1=temp; end
  path=ftv5.path_cache(tr,m,o);
  for T=[3 4.5 5 8 12]
   plan=ftv5.phase_plan(0,0,0,T); t=0:.002:T; [s,v,a]=ftv5.phase(plan,t);
   tau=ftv5.torque(path,s,v,a,m);
   fprintf('%s reverse=%d T=%.1f required_health=%.5f floor=%.2f speed_ratio=%.3f\n', ...
       profile{1},reverse,T,max(abs(tau(2,:)))/m.taumax(2),path.floor(2),max(abs(path.d*v)./m.dqmax,[],'all'));
  end
 end
end
end
