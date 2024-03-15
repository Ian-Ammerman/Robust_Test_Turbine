% Run IEA-15MW to replicate LQG paper results
clear all; close all; clc;

%% User Inputs
sim = 'DLC_1.6';
model_root = '5MW_OC4Semi_WSt_WavesWN';
tc = 10;

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

%% Form state-space object
% Full system model
sys = ss(A,B,C,D);

% Target inputs
% inputs = [1,8,9,67];
inputs = [1,8,9];

%% Reduce system order
R = reducespec(sys,'balanced');
view(R)
new_order = 13;
[rsys,info] = balred(sys,new_order);

% %% Discretize full system
% full_sysd = c2d(sys,dt,'tustin');
% [Afull,Bfull,Cfull,Dfull] = ssdata(full_sysd);
% 
% Hfull = Cfull([25,7],:); % full measurement function
% 
% % Simpilfy input vector
% Bfull = Bfull(:,inputs);
% Dfull = Dfull(:,inputs);

%% Discretize reduced system
red_sysd = c2d(rsys,dt,'tustin');
[Ared,Bred,Cred,Dred] = ssdata(red_sysd);

measurements = [25,7];
Hred = Cred(measurements,:); % reduced measurement function

% Simplify input vector
Bred = Bred(:,inputs);
Dred = Dred(:,inputs);

%% Compare Impulse Responses
% % Compare impulse responses to wave excitation
% [Y,T] = impulse(sys);
% [Yr,Tr] = impulse(rsys);

% input_num = 3;
% Y = Y(:,:,input_num);
% Yr = Yr(:,:,input_num);
% for i = 1:13
%     figure
%     gca; hold on; box on;
%     plot(T,Y(:,i),'DisplayName','Original');
%     plot(Tr,Yr(:,i),'DisplayName','Reduced');
%     legend
% end

%% Prepare Inputs
pitch_commands = (pi/180)*[sim_results.BldPitch1,sim_results.BldPitch1,sim_results.BldPitch1];
PtfmPitch = sim_results.PtfmPitch;
rot_speed = sim_results.RotSpeed;
gen_torque = sim_results.GenTq;
wind = sim_results.Wind1VelX;
wave = sim_results.Wave1Elev;

% Extra measurements
TT_acc = sim_results.YawBrTAxp;

% Low-pass filter rotor speed measurement


% Offset wave inputs by causalization time
wave_shift = dsearchn(sim_results.Time,tc);
wave = [wave(wave_shift:end);
        zeros(wave_shift,1)];

% Input operating points (from .1 file)
torque = 4.309E4;
cpitch = 3.359E-1;
uwind  = 18;

% Adjust inputs with operating point
pitch_commands = pitch_commands - cpitch;
gen_torque = gen_torque - torque;
wind = wind - uwind;
% Note: wave is OK as OP for wave is always 0

%% Prepare Kalman Filter
Rkf = 10^-5*eye(length(measurements));

%% Simulation Linear System
time = sim_results.Time;

% Simulate full system
P = zeros(size(Ared));
x = zeros(size(Ared,1),1);
Ylin = zeros(size(Cred,1),length(time));
Qkf = eye(size(Ared));

for i = 1:length(time)

    % u = [wind(i);
    %      gen_torque(i);
    %      mean(pitch_commands(i,:));
    %      wave(i)];

    u = [wind(i);
         gen_torque(i);
         mean(pitch_commands(i,:))];

    % Do prediction step
    [x,P] = predict(x,P,Ared,Qkf,Bred,u);

    % % Get measurements
    % z = [PtfmPitch(i),rot_speed(i)]';
    % 
    % % Do update step
    % [x,P,K] = update(Hfull,P,Rkf,z,x,y_OP(measurements));

    % Log outputs
    Ylin(:,i) = Cred*x + Dred*u + y_OP;

end
Ylin = Ylin';

%% Simulate reduced system
P = zeros(size(Ared));
x = zeros(size(Ared,1),1);
Yred = zeros(size(Cred,1),length(time));
Qkf = eye(size(Ared));

for i = 1:length(time)

    % u = [wind(i);
    %      gen_torque(i);
    %      mean(pitch_commands(i,:));
    %      wave(i)];

    u = [wind(i);
         gen_torque(i);
         mean(pitch_commands(i,:))];

    % Do prediction step
    [x,P] = predict(x,P,Ared,Qkf,Bred,u);

    % Get measurements
    z = [PtfmPitch(i),rot_speed(i)]';

    % Do update step
    [x,P,K] = update(Hred,P,Rkf,z,x,y_OP(measurements));

    % Log outputs
    Yred(:,i) = Cred*x + Dred*u + y_OP;

end
Yred = Yred';


%% Plot Comparison
 close all;

tc = 0;

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,25),'DisplayName','Linear Model');
plot(time-tc,Yred(:,25),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.PtfmPitch,'DisplayName','OpenFAST');
title('Platform Pitch [deg]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,7),'DisplayName','Linear Model');
plot(time-tc,Yred(:,7),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.RotSpeed,'DisplayName','OpenFAST');
title('Rotor Speed [RPM]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,51),'DisplayName','Linear Model');
plot(time-tc,Yred(:,51),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.TwrBsMyt,'DisplayName','OpenFAST');
title('Tower Base Bending Moment [kN-m]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,21),'DisplayName','Linear Model');
plot(time-tc,Yred(:,21),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.PtfmSurge,'DisplayName','OpenFAST');
title('Platform Surge [m]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,9),'DisplayName','Linear Model');
plot(time-tc,Yred(:,9),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.OoPDefl1,'DisplayName','OpenFAST');
title('Out-of-Plane Deflection [m]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,15),'DisplayName','Linear Model');
plot(time-tc,Yred(:,15),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.YawBrTAxp,'DisplayName','OpenFAST');
title('Tower-Top Acceleration [m/s^2]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,100),'DisplayName','Linear Model');
plot(time-tc,Yred(:,100),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.FAIRTEN1,'DisplayName','OpenFAST');
title('Port Mooring Tension [N]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,102),'DisplayName','Linear Model');
plot(time-tc,Yred(:,102),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.FAIRTEN2,'DisplayName','OpenFAST');
title('Lead Mooring Tension [N]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Ylin(:,104),'DisplayName','Linear Model');
plot(time-tc,Yred(:,104),'DisplayName','Kalman Filter');
plot(sim_results.Time,sim_results.FAIRTEN3,'DisplayName','OpenFAST');
title('Starboard Mooring Tension [N]')
xlabel('Time [s]')
legend


function [x,P] = predict(x,P,F,Q,B,u)
    x = F*x + B*u; %predict states
    P = F*P*F' + Q; %predict process covariance
end

% Update
function [x,P,K] = update(H,P,R,z,x,OP)
    S = H*P*H' + R; % Project system uncertainty into measurement space & add measurement uncertainty
    K = P*H'*inv(S);
    y = z-(H*x + OP); % Error term
    x = x+K*y;
    KH = K*H;
    P = (eye(size(KH))-KH)*P;
end