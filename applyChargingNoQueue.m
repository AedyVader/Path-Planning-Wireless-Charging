function applyChargingNoQueue(dt)
% APPLYCHARGINGNOQUEUE Apply wireless charging to all UAVs in-range
%
% Usage:
%   applyChargingNoQueue(dt)
%
% Input:
%   dt - time interval (seconds) for energy updates
%
% This function allows unlimited concurrent charging from multiple sources

global params_;

% Input validation
if nargin < 1 || isempty(dt) || dt <= 0
    dt = 0.1;  % Fallback
end

% Safety checks
if ~isfield(params_, 'UAV_energy') || isempty(params_.UAV_energy)
    return;
end

numUAV = params_.UAVnum;
numChargers = size(params_.mobile_chargers, 1);
map_meters = params_.map_meters;

% Track charging statistics
charging_occurred = false;
total_power_delivered = 0;

% =========================================================================
% CHARGING LOOP: Process each UAV
% =========================================================================
for u = 1:numUAV
    
    % Skip UAVs that are out of energy or completed
    if params_.UAV_energy(u) <= 0
        continue;
    end
    
    % Skip completed UAVs
    if isfield(params_, 'SumTarget') && params_.SumTarget(u) == 1
        continue;
    end
    
    % Initialize energy gain for this UAV
    total_energy_J = 0;
    uav_charged = false;
    best_charger = 0;
    best_power = 0;
    
    % Check each charger's contribution
    for c = 1:numChargers
        charger_pos = params_.mobile_chargers(c, :);
        uav_pos = params_.UAV_pos(u, :);
        
        % Calculate distance (normalized -> meters)
        d_norm = norm(uav_pos - charger_pos);
        d_m = d_norm * map_meters;
        
        % Check if within charging radius
        if d_m > params_.charging_radius * map_meters
            continue;  % Out of range
        end
        
        % Calculate received power using wireless charging model
        [P_r_W, eta_d] = wirelessCharge(params_.charger_tx_power, d_m);
        
        % DEBUG: Print charging details for first iteration
        if params_.iter <= 2 && P_r_W > 0.1
            fprintf('[CHARGE] UAV%d from Charger%d: d=%.1fm, eta=%.4f%%, P=%.1fW\n', ...
                u, c, d_m, eta_d * 100, P_r_W);
        end
        
        % Add energy if charging is significant
        if P_r_W > 0 && dt > 0
            energy_from_charger_J = P_r_W * dt;
            total_energy_J = total_energy_J + energy_from_charger_J;
            uav_charged = true;
            charging_occurred = true;
            
            % Track best charger
            if P_r_W > best_power
                best_power = P_r_W;
                best_charger = c;
            end
        end
    end
    
    % =========================================================================
    % APPLY ENERGY GAIN TO UAV
    % =========================================================================
    if total_energy_J > 0
        % Convert joules to percentage
        if ~isfield(params_, 'max_energy_J') || isempty(params_.max_energy_J)
            params_.max_energy_J = 3e6;  % Fallback
        end
        
        energy_pct = total_energy_J / params_.max_energy_J;
        
        % Store old energy for logging
        old_energy = params_.UAV_energy(u);
        
        % Add energy (cap at max)
        params_.UAV_energy(u) = min(params_.max_energy, ...
            params_.UAV_energy(u) + energy_pct);
        
        % Calculate actual gain
        actual_gain_pct = params_.UAV_energy(u) - old_energy;
        
        % Track statistics
        total_power_delivered = total_power_delivered + best_power;
        
        % DEBUG: Print significant charging events
        if actual_gain_pct > 0.001  % More than 0.1% gain
            fprintf('  ⚡ UAV%d charged: %.1f%% -> %.1f%% (+%.2f%%) from Charger%d @ %.1fW\n', ...
                u, old_energy * 100, params_.UAV_energy(u) * 100, ...
                actual_gain_pct * 100, best_charger, best_power);
        end
    end
end

% =========================================================================
% SUMMARY STATISTICS (Optional - only first few iterations)
% =========================================================================
if params_.iter <= 3
    if charging_occurred
        fprintf('  └─ Iteration %d: Total charging power = %.1f W across all UAVs\n', ...
            params_.iter, total_power_delivered);
    else
        fprintf('  └─ Iteration %d: No charging occurred (all UAVs out of range)\n', ...
            params_.iter);
    end
end

end

