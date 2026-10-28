function run_fault_pilot
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'generated'),fullfile(root,'tests')); maxNumCompThreads(1);
validate_controller;
c=ftmpc.config(); m=ftmpc.model(); kinds={'Flatness','LTV','LTI'}; profiles={'Inertia','Gravity'}; rows={};
for a=1:2
 tr=ftmpc.trajectory(profiles{a},'evaluation',m,c); fs=ftmpc.faults(tr);
 fprintf('%s ratios: %s; fault health: %s\n',profiles{a},mat2str(tr.nominal_ratios,3),mat2str(fs(:,3)',3));
 for j=1:3
  p=ftmpc.prepare(kinds{j},tr,m,c); [~,obs]=ftmpc.perturbation('nominal',1);
  r=ftmpc.simulate(p,m,m,c,fs(3,:),obs,1729); %#ok<NASGU>
  save(fullfile(root,'results',['fault_pilot_' profiles{a} '_' kinds{j} '.mat']),'r','-v7');
  row=r.metrics; row.controller=kinds{j}; row.profile=profiles{a}; rows{end+1}=row; %#ok<AGROW>
  fprintf('%s %s RMSE=%.5g fallback=%.3f iterlimit=%.3f p95=%.3fms wall=%.2fs\n',profiles{a},kinds{j},row.rmse_q,row.fallback_fraction,row.QP_iteration_limit_fraction,row.controller_p95_ms,row.wall_seconds);
 end
end
ftmpc.write_json(fullfile(root,'results','fault_pilot.json'),rows);
end
