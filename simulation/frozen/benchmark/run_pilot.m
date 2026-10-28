function run_pilot
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'generated'));
maxNumCompThreads(1);
c=ftmpc.config(); m=ftmpc.model(); kinds={'Flatness','LTV','LTI'};
tr=ftmpc.trajectory('Inertia','evaluation',m,c); fs=ftmpc.faults(tr); disp(tr.nominal_ratios); disp(fs);
[~,obs]=ftmpc.perturbation('nominal',1); rows=cell(1,3);
for j=1:3
    p=ftmpc.prepare(kinds{j},tr,m,c); r=ftmpc.simulate(p,m,m,c,fs(1,:),obs,1729);
    save(fullfile(root,'results',['pilot_' kinds{j} '.mat']),'r','-v7');
    row=r.metrics; row.controller=kinds{j}; rows{j}=row; disp(row);
end
ftmpc.write_json(fullfile(root,'results','pilot.json'),rows);
end
