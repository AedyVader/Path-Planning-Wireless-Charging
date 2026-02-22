% assignCharger.m - NO QUEUE VERSION
function [charger_idx, charger_dist] = assignCharger(uav_id, uav_pos, mobile_chargers, current_assignments)
% ASSIGNCHARGER Assign the nearest charger to a UAV for wireless charging
%
% No queueing - all UAVs within range can charge simultaneously.
% This version simply returns the nearest charger without checking
% assignment limits, allowing unlimited concurrent charging.
%
% Inputs:
%   uav_id - UAV identifier (integer)
%   uav_pos - UAV position [x, y] in normalized coordinates [0,1]
%   mobile_chargers - Matrix of charger positions (num_chargers x 2)
%   current_assignments - (unused in no-queue mode, kept for compatibility)
%
% Outputs:
%   charger_idx - Index of the nearest charger (1 to num_chargers)
%   charger_dist - Distance to that charger (normalized units)
%
% Example:
%   [idx, dist] = assignCharger(1, [0.5 0.5], [0.3 0.7; 0.8 0.2], [0; 0])
%   % Returns: idx=1, dist=0.28 (charger 1 is closer)

global params_;

% Calculate Euclidean distances to all chargers
charger_dists = vecnorm(mobile_chargers - uav_pos, 2, 2);

% Simply return the nearest charger - no queue checking
[charger_dist, charger_idx] = min(charger_dists);

% Optional: Add verbose logging if enabled
if isfield(params_, 'verbose') && params_.verbose
    if charger_dist < params_.charging_radius
        fprintf('  [ASSIGN] UAV %d → Charger %d (dist: %.4f, IN RANGE)\n', ...
                uav_id, charger_idx, charger_dist);
    end
end

% Note: In the no-queue version, we don't check or update current_assignments
% The applyChargingNoQueue function handles actual power transfer based on
% distance, allowing multiple UAVs to charge from the same charger simultaneously

end

