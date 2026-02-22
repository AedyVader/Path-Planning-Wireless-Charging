% Global parameters
global params_;             

% Initialize target (final destination)
params_.TARGET = round([0.85 0.95] * params_.N);    % Target position

for i = 1:params_.UAVnum
    params_.UAV_pos(i, :) = params_.UAV_info(i, 1:2);
end

params_.needReplan = ones(1, params_.UAVnum);   % UAVi's need to re-plan when needReplan(i) = 1
params_.SumTarget = zeros(1, params_.UAVnum);   % When UAVi's SumTarget(i) = 1, don't need further move

% Initialize obstacles
params_.obstaclesUAV = obstaclesInfo();                 % Unknown obstacles (includs all UAVs) location matrix
params_.obstaclesSize = size(params_.obstaclesUAV, 1);  % Size of obstacles  
params_.obstaclesUAV2plot = obs2plot();                 % Used when drawing obstacles in map
params_.obstacles = {};                                 % No enemy is detected initially

for i = 1:params_.UAVnum
    params_.obstacles{i} = [];
end

% Initialize trace record
params_.traceRecord = {};   % No record of trace initially

for i = 1:params_.UAVnum
    params_.traceRecord{i} = [];
end

% Initialize G Matrix (cost)
params_.G = {};     % Cost matrix
D = ones(params_.N, params_.N) * params_.N ^ 2;   % Initialize D with all elements are N ^ 2 and target 0
D(params_.TARGET(1), params_.TARGET(2)) = 0;

for i = 1:params_.UAVnum
    params_.G{i} = [D];
end


