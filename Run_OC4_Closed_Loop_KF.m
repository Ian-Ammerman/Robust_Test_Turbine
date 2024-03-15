clear all; close all; clc;

%% User Inputs
sim = 'DLC_1.6_18ms';
model_root = '5MW_OC4Semi_WSt_WavesWN';
reduced_order = 13;

%% Directories
linear_model_dir = 'C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Models\5MW_OC4Semi_WSt_WavesWN';
simulations_dir = 'C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Simulations';

%% Load OpenFAST Results
load(sprintf('%s_FAST_Results.mat',model_root));

time = sim_results.Time;
dt = mean(diff(time));

%% Load in Linear Model
load(sprintf('%s/%s_A.mat',linear_model_dir,model_root))
load(sprintf('%s/%s_B.mat',linear_model_dir,model_root))
load(sprintf('%s/%s_C.mat',linear_model_dir,model_root))
load(sprintf('%s/%s_D.mat',linear_model_dir,model_root))
load(sprintf('%s/%s_x_OP.mat',linear_model_dir,model_root))
load(sprintf('%s/%s_y_OP.mat',linear_model_dir,model_root))

%% Reduce System Order
sys = ss(A,B,C,D);

R = reducespec(sys,'balanced');
[rsys,info] = balred(sys,reduced_order);

%% Discretize reduced system
red_sysd = c2d(rsys,dt,'tustin');
[Ared,Bred,Cred,Dred] = ssdata(red_sysd);

%% Remove excessive inputs
inputs = [1,8,9]; % Wind,Tq,CP
ctrl_inputs = 8; % Collective Pitch only

Bred = Bred(:,inputs);
Bctr = Bred(:,2);
Dred = Dred(:,inputs);

Hred = Cred([35,10],:); % reduced measurement function
DHred = Dred([35,10],:); % reduced measurement function feedthrough

%% Define System Limits
Max_pitch = 90;
Min_pitch = -15;
Max_pitch_rate = 2;
Min_pitch_rate = 0;

%% Define Kalman Filter
Rkf = 10^-5*eye(2);
Qkf = eye(size(Ared));
Pkf = zeros(size(Ared));
Hkf = zeros(size(Cred));
    Hkf(35,:) = Hred(1,:);
    Hkf(10,:) = Hred(2,:);

%% Define Controller
% Q & R matrices
Qcontrol = eye(size(Ared,1));
    Qcontrol(2,2) = 1; % Pitch rotation
    Qcontrol(3,3) = 1; % Tower FA displacement
    Qcontrol(10,10) = 1; % rotor speed

Rcontrol = eye(length(ctrl_inputs));

% Define integral gain
Ki = 0.2; % rads/s, from paper

% Compute LQR gains
[Klqg,S,CLP] = dlqr(Ared,Bctr,Qcontrol,Rcontrol);

% Define reference signal
Ref = zeros(13,1);
Ref(10) = 9; % Rated RPM

% Define generator torque
GenTq = 43094;

%% Define Global Variables
global P x
P = zeros(size(Ared));
x = zeros(size(Ared,1),1);

%% Define FAST Model to Simulate
fast_model = '5MW_OC4Semi_WSt_WavesWN';
fst_file = '5MW_OC4Semi_WSt_WavesWN';

% Define variables for S-function block
FAST_InputFileName = sprintf('C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine\\Models\\%s\\%s.fst',fast_model,fst_file);
TMax               = 60; % seconds

% disp('Ready to run simulink!')
sim('OC4_Kalman_Filter.mdl',[0,TMax]);
