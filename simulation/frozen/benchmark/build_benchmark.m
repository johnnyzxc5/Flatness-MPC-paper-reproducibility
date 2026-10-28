function build_benchmark
%BUILD_BENCHMARK Generate portable-on-host MEX kernels from inspected sources.
root=fileparts(mfilename('fullpath')); addpath(root); m=ftmpc.model();
out=fullfile(root,'generated'); if ~isfolder(out), mkdir(out); end
cfg=coder.config('mex'); cfg.GenerateReport=false;
codegen('-config',cfg,'ftmpc_dynamics','-args',{zeros(6,1),zeros(6,1),m}, ...
    '-o',fullfile(out,'ftmpc_dynamics_mex'),'-d',fullfile(out,'dynamics'));
codegen('-config',cfg,'ftmpc_integrate','-args',{zeros(12,1),zeros(6,1),0,0,0,m,zeros(3,1)}, ...
    '-o',fullfile(out,'ftmpc_integrate_mex'),'-d',fullfile(out,'integrate'));
addpath(out); disp('Compiled shared dynamics and RK4 kernels.');
end
