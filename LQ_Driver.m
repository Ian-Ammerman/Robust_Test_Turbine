% Run IEA-15MW to replicate LQG paper results
clear all; close all; clc;

%% User Inputs
sim = 'Steady_12ms';
model_root = '5MW_OC3Spar_DLL_WTurb_WavesIrr';

%% Directories
linear_model_dir = 'C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Models\5MW_OC3Spar_Linear';
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

% Remove excessive inputs
inputs = [1,8,9,67];
B = B(:,inputs);
D = D(:,inputs);

% Design kalman filter
sys = ss(A,B,C,D);

R = reducespec(sys,'balanced');
view(R)
new_order = 11;
% new_order = input('New Model Order: ');
% rsys = getrom(R,Order=new_order);
[rsys,info] = balred(sys,new_order);

% % Compare impulse responses to wave excitation
% [Y,T] = impulse(sys);
% [Yr,Tr] = impulse(rsys);

% input_num = 3;
% Y = Y(:,:,input_num);
% Yr = Yr(:,:,input_num);

% Discretize full system
full_sysd = c2d(sys,dt,'tustin');
[Afull,Bfull,Cfull,Dfull] = ssdata(full_sysd);

Hfull = Cfull([35,10],:); % full measurement function

% Discretize reduced system
red_sysd = c2d(rsys,dt,'tustin');
[Ared,Bred,Cred,Dred] = ssdata(red_sysd);

Hred = Cred([35,10],:); % reduced measurement function

% %%
% for i = 1:13
%     figure
%     gca; hold on; box on;
%     plot(T,Y(:,i),'DisplayName','Original');
%     plot(Tr,Yr(:,i),'DisplayName','Reduced');
%     legend
% end

%% Prepare Inputs
pitch_commands = (pi/180)*[sim_results.BldPitch1,sim_results.BldPitch2,sim_results.BldPitch3];
PtfmPitch = sim_results.PtfmPitch;
rot_speed = sim_results.RotSpeed;
gen_torque = sim_results.GenTq;
wind = sim_results.Wind1VelX;
wave = sim_results.Wave1Elev;

%% Prepare Kalman Filter
Rkf = 10^-4*eye(2);

%% Simulation Full System
time = sim_results.Time;

% Simulate full system
P = zeros(size(Afull));
x = zeros(size(Afull,1),1);
Yfull = zeros(size(Cfull,1),length(time));
Qkf = eye(size(Afull));

for i = 1:length(time)

    u = [wind(i);
         gen_torque(i);
         mean(pitch_commands(i,:));
         wave(i)];

    % Do prediction step
    [x,P] = predict(x,P,Afull,Qkf,Bfull,u);

    % Get measurements
    z = [PtfmPitch(i),rot_speed(i)]';

    % Do update step
    [x,P,K] = update(Hfull,P,Rkf,z,x,y_OP([35,10]));

    % Log outputs
    Yfull(:,i) = Cfull*x + Dfull*u + y_OP;

end
Yfull = Yfull';

%% Simulate reduced system
P = zeros(size(Ared));
x = zeros(size(Ared,1),1);
Yred = zeros(size(Cred,1),length(time));
Qkf = eye(size(Ared));

for i = 1:length(time)

    u = [wind(i);
         gen_torque(i);
         mean(pitch_commands(i,:));
         wave(i)];

    % Do prediction step
    [x,P] = predict(x,P,Ared,Qkf,Bred,u);

    % Get measurements
    z = [PtfmPitch(i),rot_speed(i)]';

    % Do update step
    [x,P,K] = update(Hred,P,Rkf,z,x,y_OP([35,10]));

    % Log outputs
    Yred(:,i) = Cred*x + Dred*u + y_OP;

end
Yred = Yred';


%% Plot Comparison
% close all;

tc = 0;

figure
gca; hold on; box on;
plot(time-tc,Yfull(:,35),'DisplayName','Full Model');
plot(time,Yred(:,35),'DisplayName','Reduced Model');
plot(sim_results.Time,sim_results.PtfmPitch,'DisplayName','OpenFAST');
title('Platform Pitch [deg]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Yfull(:,11),'DisplayName','Full Model');
plot(time-tc,Yred(:,11),'DisplayName','Reduced Model');
plot(sim_results.Time,sim_results.GenSpeed,'DisplayName','OpenFAST');
title('Generator Speed [RPM]')
xlabel('Time [s]')
legend

figure
gca; hold on; box on;
plot(time-tc,Yfull(:,83),'DisplayName','Full Model');
plot(time-tc,Yred(:,83),'DisplayName','Reduced Model');
plot(sim_results.Time,sim_results.TwrBsMyt,'DisplayName','OpenFAST');
title('Tower Base Bending Moment [kN-m]')
xlabel('Time [s]')
legend

% figure
% gca; hold on; box on;
% plot(time-tc,Yfull(:,31),'DisplayName','Full Model');
% plot(time-tc,Yred(:,31),'DisplayName','Reduced Model');
% plot(sim_results.Time,sim_results.PtfmSurge,'DisplayName','OpenFAST');
% title('Platform Surge [m]')
% xlabel('Time [s]')
% legend


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