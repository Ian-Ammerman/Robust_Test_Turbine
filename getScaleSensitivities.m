% Script to get sensitivities
close all; clear all; clc;

%% Load Cp & Ct Information
load('C:\Umaine Google Sync\GitHub\Robust_Test_Turbine\Aerodynamics\Stiffer_Blade_Aero.mat');

%% Define Plotting Range
% Blade Pitch Range
blade_pitch = [0:0.5:10];

%% Transform Data & Plot
% Form vectors along blade pitch
CtofB = transpose(Aero.Ct);
CpofB = transpose(Aero.Cp);

% Interpolate intermediate NaN values
for i = 1:size(CtofB,2)
    CtofB(:,i) = smooth(CtofB(:,i),2);
    CpofB(:,i) = smooth(CpofB(:,i),2);
end

figure
gca; hold on; box on;
title('C_t vs Blade Pitch')
xlabel('Blade Pitch [deg]')
ylabel('C_t [-]')
for i = length(Aero.TSR(:,1)):-2:63
    plot_points = (CtofB(:,i) ~= 0);
    if min(plot_points) == 0
        first_nz = dsearchn(plot_points,0);
        plot_points(first_nz - 1) = 0;
    end
    plot(blade_pitch(plot_points),CtofB(plot_points,i),'DisplayName',sprintf('TSR = %0.3g',Aero.TSR(i,1)))
end
legend

figure
gca; hold on; box on;
title('C_p vs Blade Pitch')
xlabel('Blade Pitch [deg]')
ylabel('C_p [-]')
for i = length(Aero.TSR(:,1)):-2:63
    plot_points = (CpofB(:,i) ~= 0);
    if min(plot_points) == 0
        first_nz = dsearchn(plot_points,0);
        plot_points(first_nz - 1) = 0;
    end
    plot(blade_pitch(plot_points),CpofB(plot_points,i),'DisplayName',sprintf('TSR = %0.3g',Aero.TSR(i,1)))
end
legend