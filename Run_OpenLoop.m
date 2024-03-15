% make sure the OpenFAST directory where the FAST_SFunc.mex* file is located
% is in the MATLAB path (also make sure any other OpenFAST library files that
% are needed are on the MATLAB path)
%    (relative path names are not recommended in addpath()):
% addpath('../../../build/bin'); % install location for Windows Visual Studio builds
% addpath(genpath('../../../install')); % cmake default install location

clear all; close all; clc;

%% Define FAST Model to Simulate
fast_model = '5MW_OC4Semi_WSt_WavesWN';
fst_file = '5MW_OC4Semi_WSt_WavesWN';

% these variables are defined in the OpenLoop model's FAST_SFunc block:
FAST_InputFileName = sprintf('C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine\\Models\\%s\\%s.fst',fast_model,fst_file);
TMax               = 60; % seconds

sim('C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Simulink\OpenLoop.mdl',[0,TMax]);
