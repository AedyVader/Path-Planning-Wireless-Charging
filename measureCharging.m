function charging_metrics = measureCharging()
% MEASURECHARGING Calculate comprehensive charging performance metrics
%
% This function must be saved as measureCharging.m in your project directory
%
% Outputs:
%   charging_metrics - struct containing:
%     - total_charging_events: total number of charging instances
%     - charging_time_per_uav: time spent charging for each UAV (in seconds)
%     - charging_events_per_uav: number of charging events per UAV
%     - min_energy_reached: minimum energy level reached per UAV
%     - avg_energy: average final energy across all UAVs
%     - energy_depletion_count: number of times any UAV hit 0% energy
%     - mission_completion_rate: percentage of UAVs that completed mission
%     - avg_completion_time: average mission completion time
%     - charger_movement_distance: total distance moved by all chargers
%     - detour_distance: extra distance traveled for charging

global params_;

% Initialize metrics
charging_metrics = struct();

% 1. Mission completion metrics
if isfield(params_, 'completionTime') && isfield(params_, 'hasCompleted')
    charging_metrics.completion_times = params_.completionTime;
    completed_uavs = params_.hasCompleted > 0;
    charging_metrics.mission_completion_rate = sum(completed_uavs) / params_.UAVnum;
    
    if any(completed_uavs)
        charging_metrics.avg_completion_time = mean(params_.completionTime(completed_uavs));
    else
        charging_metrics.avg_completion_time = NaN;
    end
else
    charging_metrics.completion_times = zeros(params_.UAVnum, 1);
    charging_metrics.mission_completion_rate = 0;
    charging_metrics.avg_completion_time = NaN;
end

% 2. Energy metrics
if isfield(params_, 'UAV_energy')
    charging_metrics.avg_energy = mean(params_.UAV_energy);
    charging_metrics.final_energy_per_uav = params_.UAV_energy;
else
    charging_metrics.avg_energy = 0;
    charging_metrics.final_energy_per_uav = zeros(params_.UAVnum, 1);
end

% Calculate minimum energy reached from energy history
charging_metrics.min_energy_reached = zeros(params_.UAVnum, 1);
charging_metrics.energy_depletion_count = 0;

if isfield(params_, 'energy_history') && ~isempty(params_.energy_history)
    for u = 1:params_.UAVnum
        if ~isempty(params_.energy_history{u})
            charging_metrics.min_energy_reached(u) = min(params_.energy_history{u});
            if charging_metrics.min_energy_reached(u) <= 0
                charging_metrics.energy_depletion_count = charging_metrics.energy_depletion_count + 1;
            end
        else
            charging_metrics.min_energy_reached(u) = params_.UAV_energy(u);
        end
    end
else
    % No energy history available
    for u = 1:params_.UAVnum
        charging_metrics.min_energy_reached(u) = params_.UAV_energy(u);
    end
end

charging_metrics.min_energy = min(charging_metrics.min_energy_reached);

% 3. Charging event metrics - IMPROVED VERSION
charging_metrics.total_charging_events = 0;
charging_metrics.charging_time_per_uav = zeros(params_.UAVnum, 1);
charging_metrics.charging_events_per_uav = zeros(params_.UAVnum, 1);

% Calculate time per step (assuming constant time step)
if isfield(params_, 'dt')
    time_per_step = params_.dt;
else
    % Default estimate: if we know cruise speed and step size
    if isfield(params_, 'stepWay') && isfield(params_, 'cruise_speed')
        % stepWay is normalized, convert to time
        % stepWay * map_size / cruise_speed
        if isfield(params_, 'map_meters')
            step_distance_m = params_.stepWay * params_.map_meters;
            time_per_step = step_distance_m / params_.cruise_speed;
        else
            time_per_step = 1.0; % Default 1 second per step
        end
    else
        time_per_step = 1.0; % Default fallback
    end
end

if isfield(params_, 'traceRecord') && ~isempty(params_.traceRecord)
    for u = 1:params_.UAVnum
        if isempty(params_.traceRecord{u})
            continue;
        end
        
        trace = params_.traceRecord{u};
        charging_steps = 0;
        charging_events = 0;
        was_charging = false;
        
        % Count steps where UAV was within charging radius of any charger
        for t = 1:size(trace, 1)
            is_charging_now = false;
            
            for c = 1:size(params_.mobile_chargers, 1)
                % Get charger position at this time step
                charger_pos = getChargerPositionAtTime(c, t);
                
                % Check distance to charger at this step
                dist = norm(trace(t, :) - charger_pos);
                
                if dist < params_.charging_radius
                    is_charging_now = true;
                    charging_steps = charging_steps + 1;
                    
                    % Count as new event if wasn't charging before
                    if ~was_charging
                        charging_events = charging_events + 1;
                    end
                    
                    break; % Don't double-count if in range of multiple chargers
                end
            end
            
            was_charging = is_charging_now;
        end
        
        % Convert steps to time (seconds)
        charging_metrics.charging_time_per_uav(u) = charging_steps * time_per_step;
        charging_metrics.charging_events_per_uav(u) = charging_events;
        charging_metrics.total_charging_events = charging_metrics.total_charging_events + charging_events;
    end
else
    % If no trace record, set to zeros
    charging_metrics.charging_time_per_uav = zeros(params_.UAVnum, 1);
    charging_metrics.charging_events_per_uav = zeros(params_.UAVnum, 1);
end

% 4. Charger movement distance
if isfield(params_, 'charger_history') && ~isempty(params_.charger_history)
    charging_metrics.charger_movement_distance = 0;
    num_chargers = length(params_.charger_history);
    
    for c = 1:num_chargers
        if c <= length(params_.charger_history) && ...
           ~isempty(params_.charger_history{c}) && ...
           size(params_.charger_history{c}, 1) > 1
            
            % Calculate total distance moved by this charger
            for t = 2:size(params_.charger_history{c}, 1)
                dist = norm(params_.charger_history{c}(t, :) - params_.charger_history{c}(t-1, :));
                charging_metrics.charger_movement_distance = charging_metrics.charger_movement_distance + dist;
            end
        end
    end
    
    % Convert normalized distance to meters if needed
    if isfield(params_, 'map_meters')
        charging_metrics.charger_movement_distance = charging_metrics.charger_movement_distance * params_.map_meters;
    end
else
    charging_metrics.charger_movement_distance = 0;
end

% 5. Path deviation (detour for charging)
charging_metrics.detour_distance = calculateDetourDistance();

% 6. Average charging time
charging_metrics.avg_charging_time = mean(charging_metrics.charging_time_per_uav);

% 7. Energy efficiency
total_distance = 0;
if isfield(params_, 'traceRecord') && ~isempty(params_.traceRecord)
    for u = 1:params_.UAVnum
        if ~isempty(params_.traceRecord{u}) && size(params_.traceRecord{u}, 1) > 1
            for t = 2:size(params_.traceRecord{u}, 1)
                total_distance = total_distance + norm(params_.traceRecord{u}(t, :) - params_.traceRecord{u}(t-1, :));
            end
        end
    end
end

charging_metrics.total_distance_traveled = total_distance;
if isfield(params_, 'map_meters')
    charging_metrics.total_distance_traveled = total_distance * params_.map_meters;
end

charging_metrics.energy_per_distance = (params_.UAVnum - sum(params_.UAV_energy)) / max(total_distance, 1e-6);

% 8. Add percentage metrics for better visualization
charging_metrics.charging_time_percentage = zeros(params_.UAVnum, 1);
for u = 1:params_.UAVnum
    if charging_metrics.completion_times(u) > 0
        charging_metrics.charging_time_percentage(u) = ...
            (charging_metrics.charging_time_per_uav(u) / charging_metrics.completion_times(u)) * 100;
    end
end

% 9. Print per-UAV summary (for debugging)
if isfield(params_, 'verbose') && params_.verbose
    fprintf('\n=== Per-UAV Charging Summary ===\n');
    for u = 1:params_.UAVnum
        fprintf('UAV %d: %.1f sec charging, %d events, Min Energy: %.1f%%\n', ...
            u, charging_metrics.charging_time_per_uav(u), ...
            charging_metrics.charging_events_per_uav(u), ...
            charging_metrics.min_energy_reached(u) * 100);
    end
end

end

%==========================================================================
% HELPER FUNCTIONS
%==========================================================================

function charger_pos = getChargerPositionAtTime(charger_idx, time_step)
% Get the position of a charger at a specific time step
global params_;

if isfield(params_, 'charger_history') && ...
   charger_idx <= length(params_.charger_history) && ...
   ~isempty(params_.charger_history{charger_idx})
    
    history = params_.charger_history{charger_idx};
    
    % If time step is within history, return that position
    if time_step <= size(history, 1)
        charger_pos = history(time_step, :);
    else
        % Return last known position
        charger_pos = history(end, :);
    end
else
    % No history available, return current position
    charger_pos = params_.mobile_chargers(charger_idx, :);
end
end

function detour_distance = calculateDetourDistance()
% Calculate extra distance traveled due to charging detours
global params_;

detour_distance = 0;

if ~isfield(params_, 'traceRecord') || isempty(params_.traceRecord)
    return;
end

for u = 1:params_.UAVnum
    if isempty(params_.traceRecord{u}) || size(params_.traceRecord{u}, 1) < 2
        continue;
    end
    
    % Calculate actual path length
    actual_path_length = 0;
    trace = params_.traceRecord{u};
    
    for t = 2:size(trace, 1)
        actual_path_length = actual_path_length + norm(trace(t, :) - trace(t-1, :));
    end
    
    % Calculate direct distance from start to target
    start_pos = params_.UAV_info(u, 1:2);
    target_pos = params_.TARGET / params_.N;
    direct_distance = norm(target_pos - start_pos);
    
    % Detour is the difference
    detour_distance = detour_distance + max(0, actual_path_length - direct_distance);
end

% Average across UAVs
if params_.UAVnum > 0
    detour_distance = detour_distance / params_.UAVnum;
end

% Convert to meters if needed
if isfield(params_, 'map_meters')
    detour_distance = detour_distance * params_.map_meters;
end

end

