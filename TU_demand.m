function TU_demand_matrix = TU_demand
% TU_DEMAND  Vectorized calculation of TU accumulated demand over an N2 x N2 grid.
%
%   TU_demand_matrix = TU_demand
%
% Uses global params_ to read params_.TU_info, params_.serviceRad, params_.N2,
% params_.n and params_.B. Returns an N2-by-N2 matrix where each cell holds
% the summed contribution of all TUs within serviceRad of that cell center.

global params_;

% grid coordinates in normalized [0,1]
N2 = params_.N2;
[xg, yg] = meshgrid((1:N2)/N2, (1:N2)/N2);

TU_demand_matrix = zeros(N2, N2);

% Precompute constants
serviceRad = params_.serviceRad;
n = params_.n;
B = params_.B;

% Loop over TUs (this loop is over number of TUs, typically small)
TU_info = params_.TU_info;
numTU = size(TU_info, 1);

for k = 1:numTU
    tx = TU_info(k, 1);
    ty = TU_info(k, 2);
    d_val = TU_info(k, 3);  % service demand scalar

    % compute mask for cells within serviceRad
    dist_mat = sqrt( (xg - tx).^2 + (yg - ty).^2 );
    mask = dist_mat <= serviceRad;

    if any(mask(:))
        % contribution function (sigmoid-like as in original)
        contrib = (1 - exp(-power(d_val, n) / (d_val + B)));
        % add contribution to those grid cells
        TU_demand_matrix(mask) = TU_demand_matrix(mask) + contrib;
    end
end

end
