function TU_demand_matrix = TU_demand_linear

% Global parameters
global params_;

% Normalize
maxDemand = max(params_.TU_info(:, 3));
minDemand = min(params_.TU_info(:, 3));

% Calculate accumulated TU_info service demand matrix
TU_demand_matrix = zeros(params_.N2, params_.N2);  % Initial T as N2 * N2 0 matrix
for x = 1:params_.N2  % For point (x,y) in N2 * N2, summarize demand from TU_info with in SERVICE_RADIS, then return T
    for y = 1:params_.N2
        sum = 0;
        for i = 1:size(params_.TU_info, 1)
            if norm([x / params_.N2,y / params_.N2] - params_.TU_info(i, 1:2)) <= params_.serviceRad
                sum = sum + (params_.TU_info(i, 3) - minDemand) / (maxDemand - minDemand);
            end
        end
        TU_demand_matrix(x, y) = sum;
    end
end

end