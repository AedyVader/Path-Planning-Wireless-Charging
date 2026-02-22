% measure.m
% This function measures the path length, service rate and risk exposure for each UAVs

function [PL, ServiceRate, Risk] = measure

% Global parameters
global params_;

% Calculate path length
PathLength = [];

for i = 1:params_.UAVnum
    PathLength(i) = 0;
    if ~isempty(params_.traceRecord{i})
        for j = 2:size(params_.traceRecord{i}, 1)
            PathLength(i) = PathLength(i) + norm(params_.traceRecord{i}(j, :) - params_.traceRecord{i}(j - 1, :));
        end
    else
        PathLength(i) = 0;
    end
end

PL = round(sum(PathLength) / params_.UAVnum, 4);

% Calculate service rate
Served = 0;
Sum = 0;
TU_info_ori = getTU_info;

for i = 1:size(params_.TU_info, 1)
    Served = Served + params_.TU_info(i, 3);
    Sum = Sum + TU_info_ori(i, 3);
end

ServiceRate = round(1 - Served / Sum, 4);

% Calculate risk
DangerMeasure = [];
E_matrix = getEmatrix(params_.N2, params_.obstaclesUAV2plot);   % Risk matrix for static obstacles

for i = 1:params_.UAVnum
    DangerMeasure(i) = 0;
    if ~isempty(params_.traceRecord{i})
        for j = 1:size(params_.traceRecord{i}, 1)
            x = round(params_.traceRecord{i}(j, 1) * params_.N2);
            y = round(params_.traceRecord{i}(j, 2) * params_.N2);

            % Ensure indices are within bounds
            x = max(1, min(x, params_.N2));
            y = max(1, min(y, params_.N2));

            % Add risk from obstacles
            static_risk = E_matrix(x, y);

            % Combine risks (for example, using a weighted sum or multiplication)
            combined_risk = static_risk;  % You can modify this combination as needed
            DangerMeasure(i) = DangerMeasure(i) + combined_risk;
        end
    else
        DangerMeasure(i) = 0;
    end
end

Risk = round(sum(DangerMeasure) / params_.UAVnum, 4);

% Print results
fprintf('Average path length = %.4f \n', PL);
fprintf('Service rate = %.4f \n', ServiceRate);
fprintf('Average risk = %.4f \n', Risk);

end
