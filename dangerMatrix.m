function tempA = dangerMatrix(s, E_matrix)

% DANGERMATRIX  Compute weight matrix A for planning using a sampled
% line-integral approximation of static risk plus demand term.
%
%   tempA = dangerMatrix(s, E_matrix)
%
% Inputs:
%   s        - current grid location [i j] in 1..N
%   E_matrix - N2 x N2 static risk matrix (values in [0,1])
%
% Output:
%   tempA    - N x N matrix of weights used by planning.m
%
% Notes:
%   - This function approximates the integral of risk along the straight
%     line from s/N to each grid cell center in the N x N planning grid
%     by sampling a small number of points along the segment and averaging
%     the E_matrix values at those points.
%   - The original algorithm advanced along the segment in tiny steps.
%     Here we use a sampled mean (nSamples) which is much faster and
%     accurate for reasonably smooth E_matrix fields.
%
% Global:
%   params_  - many parameters (N, N2, K, M, TU_demand_matrix, etc.)

global params_;

% Grid sizes
N = params_.N;
N2 = size(E_matrix, 1);  % should equal params_.N2

% Precompute normalized coordinates of planning grid cell centers (1..N)
[xIdx, yIdx] = meshgrid(1:N, 1:N);            % indices on N grid
x_norm = (xIdx - 0.5) / N;  % center positions in normalized [0,1]
y_norm = (yIdx - 0.5) / N;

% Flatten for vectorized processing
numPts = N * N;
s2_x = reshape(x_norm, [1, numPts]);   % 1 x P
s2_y = reshape(y_norm, [1, numPts]);   % 1 x P

% Current position in normalized coordinates
s1 = s / N;    % s is [i j] in 1..N -> normalized

% Euclidean distance from s to each planning grid point (normalized units)
dists = sqrt( (s1(1) - s2_x).^2 + (s1(2) - s2_y).^2 );   % 1 x P

% Keep a copy of the "base distance" (equivalent to previous tempA initialization)
base_dis = dists;   % used later: tempA = K*delta + M/(1+Demand) + base_dis

% If all distances are zero (s equals every point only when N==1), handle
if numPts == 1 && dists == 0
    % trivial case
    tempA = zeros(N, N);
    return;
end

% ----- Sampled approximation of line integral of E_matrix along each segment -----
% Number of samples along each segment (tradeoff speed vs accuracy)
nSamples = 8;

% Sample t positions (avoid t=0 to skip the source singularity if any)
t_samples = linspace(0, 1, nSamples);

% Prepare arrays for interpolation into E_matrix (E defined on [1..N2] grid)
% Coordinates in E_matrix are in normalized domain [0,1] scaled to [1..N2]
% We'll use interp2 with grid 1:N2 and map normalized positions to xi = pos*N2
xi_all = zeros(nSamples, numPts);
yi_all = zeros(nSamples, numPts);

for k = 1:nSamples
    t = t_samples(k);
    % point along segment: p = s1 + t*(s2 - s1)
    px = s1(1) + t * (s2_x - s1(1));
    py = s1(2) + t * (s2_y - s1(2));
    % map to E_matrix index space [1..N2]
    xi_all(k, :) = min(max(px * N2, 1), N2);
    yi_all(k, :) = min(max(py * N2, 1), N2);
end

% Interpolate E_matrix at all sample points (vectorized)
% build grid coordinates for interp2: X = 1:N2, Y = 1:N2
[Xgrid, Ygrid] = meshgrid(1:N2, 1:N2);

% For speed: reshape E_matrix so interp2 works in vectorized mode
% interp2 expects arrays of same size; we pass xi_all and yi_all to get values
E_samples = zeros(nSamples, numPts);
for k = 1:nSamples
    % interp2 takes (X,Y,V,Xq,Yq)
    E_samples(k, :) = interp2(Xgrid, Ygrid, E_matrix, xi_all(k, :), yi_all(k, :), 'linear', 0);
end

% Mean risk along each segment (per target cell)
mean_risk_along = mean(E_samples, 1);   % 1 x P

% Approximate integral: delta ≈ distance * mean_risk_along
delta = dists .* mean_risk_along;   % 1 x P

% ----- Demand lookup: sample TU demand at each s2 point using interp2 -----
% Use params_.TU_demand_matrix for demand values (N2 x N2)
TU_T = params_.TU_demand_matrix;
% Map normalized s2 positions to N2 grid index space
xi_demand = min(max(s2_x * N2, 1), N2);
yi_demand = min(max(s2_y * N2, 1), N2);
% Interpolate demand
demand_vals = interp2(Xgrid, Ygrid, TU_T, xi_demand, yi_demand, 'linear', 0); % 1 x P

% ----- Combine terms to produce tempA values (vectorized) -----
K = params_.K;
M = params_.M;

tempA_vec = K .* delta + M ./ (1 + demand_vals) + base_dis;  % 1 x P

% Reshape back to NxN
tempA = reshape(tempA_vec, [N, N]);

% Ensure non-negative
tempA = max(tempA, 0);

end
