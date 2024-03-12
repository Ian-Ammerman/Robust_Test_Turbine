% Compute scaled operating parameters for stiffer blades
clear all; close all; clc;

load('C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Aerodynamics\Stiffer_Blade_Aero.mat');

%% User Inputs
% Scaling parameter
lambda = 100;

% Target wind speeds
wind_targets = [8,12,18,20,22,24];

% Define scale operating point
tsr_targets = [6:12];
beta = [0:0.5:10];

% Model dimensions
R = 1.432068311; % blade length
Arotor = pi*R^2;

% Environmental constants
rho_air = 1.225; % kg/m^3

% Array initialization
thrust_targets = zeros(size(wind_targets));
Ct_targets = zeros(length(beta),1);
u_targets = zeros(length(beta),length(thrust_targets));
RPM_targets = zeros(length(beta),length(thrust_targets),length(tsr_targets));

%% Load in Blade Cp & Ct Curve Info
load('C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Aerodynamics\Stiffer_Blade_Aero.mat');

for k = 1:length(tsr_targets)
    
    % Set operating tsr
    tsr = tsr_targets(k);
    
    % Get Target Thrust Loads
    for i = 1:length(wind_targets)
        sim_case = sprintf('Steady_%ims',wind_targets(i));
        load(sprintf(['C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine\\Simulations\\%s\\' ...
                      'Turbine_asBuilt_FAST_Results.mat'],sim_case));
        % load(sprintf(['C:\\Umaine Google Sync\\GitHub\\Robust_Test_Turbine\\Simulations\\%s\\' ...
        %               '5MW_OC3Spar_DLL_WTurb_WavesIrr_FAST_Results.mat'],sim_case));

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
    end
    
    % Get Ct Targets
    % Get column associated with TSR target
    row = dsearchn(Aero.TSR(:,1),tsr);
    
    % Get Ct associated with beta/tsr pair
    for i = 1:length(beta)
        Ct_targets(i) = Aero.Ct(row,i);
    end
    
    % Get Scale Wind Targets
    for i = 1:length(beta)
        for j = 1:length(thrust_targets)
            u_targets(i,j) = sqrt((2*thrust_targets(j))/(rho_air*Arotor*Ct_targets(i)));
        end
    end
    
    % Compute RPM Target
    
    for i = 1:length(beta)
        for j = 1:length(thrust_targets)
            RPM_targets(i,j,k) = ((tsr*u_targets(i,j))/R)*(60/(2*pi));
        end
    end 
end

%% Plot RPM Target vs Beta
figure('Position',[1923.4,700.2,1074.4,1131.2])
for i = 1:size(RPM_targets,2)
    subplot(3,2,i)
    gca; hold on; box on;
    yline(120,'LineStyle','--','Color','Black','HandleVisibility','off')
    text(9.9, 120, '120 RPM', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
    for j = 1:size(RPM_targets,3)
        plot(beta,RPM_targets(:,i,j),'DisplayName',sprintf('TSR = %0.3g',tsr_targets(j)));
    end
    xlabel('Blade Pitch [deg]')
    ylabel('RPM')
    title({sprintf('u_{full} = %0.3g',wind_targets(i))})
    legend('Location','eastoutside')
end
sgtitle({'Operating RPM for Thrust Matching','1:100 IEA-15MW'});

% %% Get RPM Targets for Fixed Blade Pitch
% % Define fixed pitch
% pitch = 6.5;
% ind = dsearchn(beta',pitch);
% 
% for i = 1:size(RPM_targets,2)
%     RPM_fixed(i) = RPM_targets(ind,i);
% end



