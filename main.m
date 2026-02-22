% main.m

clc;
clear;
close all;

% Global parameters
global params_;

% Number of runs
num_runs = 1;

for run = 1:num_runs
    %% Parameter settings
    params_.Nobs = 15;       % Number of obstacles     
    params_.NT = 100;
    params_.Nfe = params_.NT * 3;
    params_.radius = 0.1;    % Radius
    params_.x_min = 0;       % Minimum boundaries of x 
    params_.x_max = 4000;    % Maximum boundaries of x 
    params_.y_min = 0;       % Minimum boundaries of y
    params_.y_max = 4000;    % Maximum boundaries of y
    params_.imgnum = 0;      % Number of looping
    params_.iter = 0;        % Number of iterations
    params_.plotFigure = 1;  % 1 - Plotting, 0 - No plotting (Used to plotting figure / graph)
    params_.isSigmoid = 1;   % 1 - Sigmoid, 0 - Linear
    
    % Adaptive parameters
    params_.K = 2.00;        % Risk tolerance coefficient
    params_.M = 5.00;        % Priority of service demand coefficient

    % Map information
    params_.N = 40;                 % Divide [0, 1] * [0, 1] map into N * N grid
    params_.N2 = 100;               % Divide [0, 1] * [0, 1] map into N2 * N2 grid when calculating risk weight matrix
    params_.episodeNum = 20 * params_.N;    % Number of episode
    params_.n = 2;                  % Parameter in sigmoid demand function
    params_.B = 8;                  % Parameter in sigmoid demand function
    params_.TU_num = 30;
    params_.TU_info = getTU_info;   % Terminal unit (TUs) information location matrix

    % UAV information
    params_.UAV_info = UAV_initialize;          % UAVs location matrix                   
    params_.UAVnum = size(params_.UAV_info,1);  % Number of UAV
    params_.UAV_pos = [];                       % UAVi's initial position
    params_.obserRad = 0.2;          % Observation radius 0 < x < 1
    params_.serviceRad = 0.2;        % Service radius within which a TU can be served
    params_.stepWay = 0.02;          % Length of one UAV movement

    % Wireless charging parameters
    params_.num_chargers = 2;                       % Number of mobile chargers
    params_.mobile_chargers = [0.35 0.45; 0.70 0.65];   % Initial charger positions 
    params_.charging_radius = 0.1;                  % Charging radius
    params_.charger_speed = 0.05;                   % Charger speed normalized (~50 m/s)
    params_.max_energy = 1;                         % Maximum energy capacity 100 percent (normalized)
    params_.charging_threshold = 0.6;               % When UAVs seek charging 
    params_.energy_consumption_rate = 0;            % Energy consumption per distance (legacy)
    params_.charging_rate = 0;                      % Energy replenishment rate (legacy)
    params_.UAV_energy = params_.max_energy * ones(params_.UAVnum, 1);   % Initialize all UAVs
    params_.min_charger_distance = 0.1;             % Minimum distance between chargers
    params_.charger_assignment_policy = 'nearest';  % 'nearest', 'rl', or 'hybrid'
    params_.rl_exploration_rate = 0.1;              % Chance of random exploration
    params_.verbose = true;                         % Enable warnings
    params_.enforce_min_distance = true;            % Enable distance constraint
    params_.charger_tx_power = 5000;                % transmitter power P (5 kW)   
    params_.power_transmission_coeff = 1;           % beta
    params_.d_eps = 1e-3;                           % small epsilon
    params_.R_max = 0.1;                            % effective charging range (normalized)
    params_.pathloss_factor = 0.0;                  % additional pathloss term
    params_.d_ref = 1;                              % reference distance (m)
    
    % --- Wireless charging efficiency constants ---
    % η(d) = ER_a * exp(-ER_b * d^ER_c)
    params_.ER_a = 1;            % Base efficiency (0–1)
    params_.ER_b = 0.00001;      % Exponential decay rate with distance
    params_.ER_c = 2;            % Power of distance term (1 = exp, 2 = Gaussian-like)

    % Charger system variables
    params_.current_assignments = zeros(size(params_.mobile_chargers, 1), 1);
    params_.max_assignments = Inf;         % unlimited number of UAVs can be "served" at once
    params_.use_queue = false;             % explicit flag for legacy checks
    params_.allow_unlimited_charging = true; % new flag used by charging module

    %% --- Physical / energy model parameters ---
    % These defaults are required by the energy/charging integration. Tune as needed.
    if ~isfield(params_, 'use_simple_energy_model')
        params_.use_simple_energy_model = false; % false => use physics-based model
    end
    if ~isfield(params_, 'map_meters')
        params_.map_meters = params_.x_max - params_.x_min;
    end
    if ~isfield(params_, 'cruise_speed')
        params_.cruise_speed = 5;  % m/s
    end
    if ~isfield(params_, 'max_energy_J')
        %params_.max_energy_J = 1e6;
        params_.max_energy_J = 5e6;      % 5 MJ battery
    end

    % Physical / energy model defaults
    params_.rho = 1.225;                 % air density (kg/m^3)
    params_.fuselage_area = 0.0151;      % fuselage equivalent flat plate area (m²)
    params_.Cd_fuselage = 0.6;           % UAV drag coefficient
    params_.P_profile_hover = 580.65;    % rotor blade profile power in hover (W)
    params_.P_induced_hover = 790.6715;  % induced power during hover (W)
    params_.v_induced_avg = 120;         % average induced velocity at rotor tip (m/s)
    params_.U_tip = 4.03;                % rotor tip speed during hover (m/s)
    params_.rotor_solidity = 0.05;       % rotor solidity
    params_.rotor_disc_area = 0.503;     % rotor disc area (m²)
    params_.incremental_correction_factor = 0.1; % Incremental correction factor
    params_.profile_coeff = 0.012;       % profile drag coefficient (used for speed correction)
    params_.rotor_radius = 0.4;          % rotor radius m (if needed)
    params_.mass_total = 10;             % total UAV mass (kg)
    params_.g = 9.81;                    % gravitational acceleration (m/s²)

    % Store original parameters
    num_chargers_list = [1, 2, 3, 4, 5];  % Test different counts
    results_chargers = struct();

    for nc_idx = 1:length(num_chargers_list)
        params_.num_chargers = num_chargers_list(nc_idx);
    
        % Generate charger positions based on count
        params_.mobile_chargers = generateChargerPositions(params_.num_chargers);
    
        % Reset UAVs and run simulation
        params_.UAV_energy = params_.max_energy * ones(params_.UAVnum, 1);
        params_.iter = 0;
        
        tic;
        fprintf('K = %.1f, M = %.3f \n', params_.K, params_.M);

        if(params_.isSigmoid == 1)
            fprintf('Using sigmoid demand function.\n');
            params_.TU_demand_matrix = TU_demand;           % Terminal units (TUs) service demand weight matrix
        else
            fprintf('Using linear demand function.\n')
            params_.TU_demand_matrix = TU_demand_linear;    % Terminal units (TUs) service demand weight matrix
        end
        
        initialize;
        drawBackground;
        main_UAVs5;
    
        % Store results
        [PL, ServiceRate, Risk] = measure;
        results_chargers(nc_idx).num_chargers = params_.num_chargers;
        results_chargers(nc_idx).path_length = PL;
        results_chargers(nc_idx).service_rate = ServiceRate;
        results_chargers(nc_idx).risk = Risk;
        results_chargers(nc_idx).completion_times = params_.completionTime;
        results_chargers(nc_idx).total_iterations = params_.iter;
        results_chargers(nc_idx).avg_energy = mean(params_.UAV_energy);
    end

    % Save and plot results
    save('charger_count_study.mat', 'results_chargers');
    plotChargerCountImpact(results_chargers);
    
%     %% Run simulation
%     tic;
%     fprintf('K = %.1f, M = %.3f \n', params_.K, params_.M);
% 
%     if(params_.isSigmoid == 1)
%         fprintf('Using sigmoid demand function.\n');
%         params_.TU_demand_matrix = TU_demand;           % Terminal units (TUs) service demand weight matrix
%     else
%         fprintf('Using linear demand function.\n')
%         params_.TU_demand_matrix = TU_demand_linear;    % Terminal units (TUs) service demand weight matrix
%     end
% 
%     COUNT = zeros(1, size(params_.TU_info, 1)); % Count each TU service time (script-level variable)
%     initialize;                                 % Initialize target, UAVs, obstacles, cost matrix
%     drawBackground;                             % Draw background map including UAVs and obstacles position 
%     main_UAVs5;                                 % UAVs movement / trajectory path
% 
%     % Print results
%     [PL, ServiceRate, Risk] = measure;     % Calculate final results

end
