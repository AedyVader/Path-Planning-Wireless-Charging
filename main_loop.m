clc;
clear;
close all;

tic;

% Global parameters
global params_;

% Dynamic obstacle parameters
params_.Nobs = 6;           % Number of obstacles     
params_.NT = 80;
params_.Nfe = params_.NT * 3;
params_.radius = 0.2;       % Radius
params_.x_min = 0;          % Minimum boundaries of x
params_.x_max = 20;         % Maximum boundaries of x
params_.y_min = 0;          % Minimum boundaries of y
params_.y_max = 20;         % Maximum boundaries of y
params_.imgnum = 0;         % Number of looping
params_.iter = 0;           % Number of iterations

% Customized parameters
params_.k_list = [10.0];
params_.m_list = [0.01, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00];
params_.plotFigure = 0;     % 1 - Plotting, 0 - No plotting
params_.isSigmoid = 0;      % 1 - Sigmoid, 0 - Linear

% Map information
params_.N = 20;
params_.N2 = 50;
params_.episodeNum = 20 * params_.N;
params_.n = 2;
params_.B = 8;

% UAV information
params_.obserRad = 0.2;
params_.serviceRad = 0.2;
params_.stepWay = 0.02;

% Data recorder
params_.pathLength_list = [];
params_.QoS_list = [];
params_.Risk_list = [];

% Run simulation
for index1 = 1:length(params_.k_list)
    for index2 = 1:length(params_.m_list)
        params_.K = params_.k_list(index1);
        params_.M = params_.m_list(index2);
        
        fprintf('\nK = %.1f, M = %.3f \n', params_.K, params_.M);
        
        % Unit loop
        params_.TU_info = getTU_info;
        
        if(params_.isSigmoid == 1)
            fprintf('Using sigmoid demand function.\n');
            params_.TU_demand_matrix = TU_demand;
        else
            fprintf('Using linear demand function.\n')
            params_.TU_demand_matrix = TU_demand_linear;
        end
        
        COUNT = zeros(1, size(params_.TU_info, 1));     % Count each TU service time
        initialize;
        drawBackground;
        main_UAVs;
        
        [PL, ServiceRate, Risk] = measure;
        params_.pathLength_list(end + 1) = PL;
        params_.QoS_list(end + 1) = ServiceRate;
        params_.Risk_list(end + 1) = Risk;        
    end
end

% Save results
saveData(params_.pathLength_list, params_.QoS_list, params_.Risk_list, params_.isSigmoid)

toc;