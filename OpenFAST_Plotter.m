% Generic plotting script
clear all; close all; clc;

%% ---------- USER INPUTS ---------- %%
% Note: This file must be run with the desired simulation directory open!!!

% Models to plot
plot_models = {'Turbine_asBuilt'};

% Channels to plot
plot_channels = {'GenSpeed'};

% Time range to plot
t_range = [0,100];

%% ---------- END USER INPUTS ---------- %%




%% ---------- BEGIN CODE ---------- %%

% Load in results
for model = 1:length(plot_models)

    % Results to load
    current_results = sprintf('%s_FAST_Results.mat',plot_models{model});
    
    % Load in results
    load(current_results);

    % Store in larger structure
    results.(plot_models{model}) = sim_results;

    % Clear out individual sim results
    clear sim_results

end

% Loop over channels to plot
for f = 1:length(plot_channels)
    
    % Generate figure
    figure
    gca; hold on; box on;

    % Loop over models & plot
    for m = 1:length(plot_models)
        plot(results.(plot_models{m}).Time,...
             results.(plot_models{m}).(plot_channels{m}),...
             'DisplayName',plot_models{m});
    end

    % Add legend & labels
    title(sprintf('%s',plot_channels{f}))
    xlabel('Time [s]')
    legend('Interpreter','none')

end