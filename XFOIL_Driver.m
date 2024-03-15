% Driver for running XFOIL analysis
clear all; close all; clc;

% Compute reynolds number
chord_length = 0.1; % m
rho = 1.225; % kg/m^3
u = 3.5; % m/s
mu = 1.825E-5;
ReynoldsNumber = (rho*u*chord_length)/mu;


% Define the path to your airfoil data file and the Reynolds number
airfoilFilePath = 'AG.dat'; % Update this path
% ReynoldsNumber = 1e6; % Example Reynolds number, adjust as needed

% Define the name of the command file for XFOIL and the output file for polar data
xfoilCmdFile = 'xfoil_commands.txt';
xfoilPolarFile = 'AG_results.dat';

% Define the angles of attack range
alphaStart = -5; % Starting angle of attack
alphaEnd = 15;   % Ending angle of attack
alphaStep = 0.5; % Step between angles

% Write the XFOIL commands to a file
fid = fopen(xfoilCmdFile, 'w');
fprintf(fid, 'LOAD %s\n', airfoilFilePath);
fprintf(fid, 'PLOP\nG\n\n'); % Plot options: turn off grid
fprintf(fid, 'PANE\n'); % Plot paneled geometry
fprintf(fid, 'OPER\n'); % Enter operating point mode
fprintf(fid, 'VISC %f\n', ReynoldsNumber); % Set Reynolds number
fprintf(fid, 'PACC\n'); % Initialize polar accumulation
fprintf(fid, '%s\n', xfoilPolarFile); % Specify polar output file
fprintf(fid, '\n'); % No dump file
fprintf(fid, 'ASEQ %f %f %f\n', alphaStart, alphaEnd, alphaStep); % Run angle of attack sequence
fprintf(fid, 'PACC\n'); % Close polar accumulation
fprintf(fid, '\n\n\n\n');
fprintf(fid, 'QUIT\n'); % Exit XFOIL
fclose(fid);

% Execute XFOIL with the command file
system(sprintf('xfoil < %s', xfoilCmdFile));

% Read the table of results from the file, skipping the header rows
polardata = readmatrix(xfoilPolarFile);
last_val = dsearchn(polardata(:,1),alphaEnd);
polardata = polardata(1:last_val,:);

% Form table
VariableNames = {'alpha', 'CL', 'CD', 'CDp', 'CM', 'Top_Xtr', 'Bot_Xtr'};
polarData = array2table(polardata,'VariableNames',VariableNames);

% Extract alpha, Cl, and Cd
alpha = polarData.alpha;
Cl = polarData.CL;
Cd = polarData.CD;

% Plot Cl and Cd versus alpha
figure;
yyaxis left;
plot(alpha, Cl, '-b');
ylabel('C_L');

yyaxis right;
plot(alpha, Cd, '-r');
ylabel('C_D');

xlabel('Angle of Attack (deg)');
title('C_L and C_D vs. Angle of Attack');
legend('C_L', 'C_D', 'Location', 'best');
grid on;
