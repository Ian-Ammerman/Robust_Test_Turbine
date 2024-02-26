% Compute scaled operating parameters for stiffer blades
clear all; close all; clc;

%% User Inputs
% Scaling parameter
lambda = 70;

% Target wind speeds
wind_targets = [8,12,18,20,22,24];

% Define scale operating point
tsr = 10.0477;
Ct = 0.2328;

% Model dimensions
R = 1.432068311; % blade length
Arotor = pi*R^2;

% Environmental constants
rho_air = 1.225; % kg/m^3

%% Load in Blade Cp & Ct Curve Info
load('C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Aerodynamics\Stiffer_Blade_Aero.mat');

%% Get Target Thrust Loads
thrust_targets = zeros(size(wind_targets));
for i = 1:length(wind_targets)
    sim_case = sprintf('Steady_%ims',wind_targets(i));
    load(sprintf('C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine\\Simulations\\%s\\Turbine_asBuilt_FAST_Results.mat',sim_case));

    trim_time = 100;
    trim_index = dsearchn(sim_results.Time,trim_time);
    
    sim_vars = fieldnames(sim_results);
    for j = 1:length(sim_vars)
        sim_results.(sim_vars{j}) = sim_results.(sim_vars{j})(trim_index:end);
    end
    
    % Compute mean thrust load for FOCAL C1
    target_thrust_full = mean(sim_results.RotThrust)*10^3;
    target_thrust_scaled = target_thrust_full/lambda^(3);

    % Store thrust target
    thrust_targets(i) = target_thrust_scaled;

    % Store target scale wind speed
    scale_wind_targets(i,1) = sqrt((2*thrust_targets(i))/(rho_air*Arotor*Ct));
end

%% Compute Necessary Operating RPM To Match Thrust
RPM_targets = ((tsr*scale_wind_targets)./R)*(60/(2*pi));