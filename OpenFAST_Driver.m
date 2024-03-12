% Basic OpenFAST driver for new turbine testing models
clear all; close all; clc;

%% ---------- USER INPUTS ---------- %%
home_dir = 'C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine';

% Model to test (must match folder name in 'Models' folder)
model = '5MW_OC3Spar_DLL_WTurb_WavesIrr';
fst_name = '5MW_OC3Spar_DLL_WTurb_WavesIrr';

% Simulation folder for outputs
sim_folder = 'Steady_8ms';

% Define Steady Wind Input

% OpenFAST version (name of corresponding folder in 'bin')
version = 'v3_5_2';
bin_name = 'openfast_x64.exe';

%% ---------- END USER INPUTS ---------- %%

%% ---------- BEGIN CODE ---------- %%
% Ensure starting from home dir
if ~strcmp(pwd,home_dir)
    cd(home_dir)
end

% Directory Definitions (relative to Robust_Test_Turbine repo)
bin_dir = sprintf('bin/%s/%s',version,bin_name);
model_dir = sprintf('Models/%s/%s.fst',model,fst_name);
sim_dir = sprintf('Simulations/%s',sim_folder);

% Prepare simulation directory
cd('Simulations')

if ~exist(sim_folder,'dir')
    mkdir(sim_folder)
end

cd('..')

% Run OpenFAST Simulation
name = sprintf('"%s/%s" "%s/%s"', home_dir, bin_dir, home_dir, model_dir);
[status,results] = system(name,'-echo');

% Relocate output files to simulation directory
disp('---------- Relocating Output Files -----------')
cd(sim_dir)

filetypes = {'.out','.ech','.sum','.outb'};
for i = 1:length(filetypes)
    try
        movefile(sprintf('../../Models/%s/*%s',model,filetypes{i}));
        fprintf('Output files of type %s relocated. \n',filetypes{i});
    catch
        fprintf('No files of type %s detected. \n',filetypes{i});
    end
end

% Process output files & format as structure (assumes .out format)
[sim_results,units] = readFastTabular(sprintf('%s.out',fst_name));

% Save results structure named after model used
save_name = sprintf('%s_FAST_Results.mat',fst_name);
save(save_name,'sim_results');

% Relocate
fclose('all');

