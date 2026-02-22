% Global parameters
global params_;

% Tracking variables
params_.replanCount = zeros(params_.UAVnum, 1);         % Replanning counter
params_.collisionAvoidance = zeros(params_.UAVnum, 1);  % Collision avoidance counter
params_.startTime = tic;                                % Start mission timer
params_.completionTime = zeros(params_.UAVnum, 1);      % Initialize completion times to 0
params_.hasCompleted = zeros(params_.UAVnum, 1);        % Initialize completion flags to 0 (false)
params_.SumTarget = zeros(params_.UAVnum, 1);           % Ensure this is initialized if not already

% Initialize charging system variables
if ~isfield(params_, 'current_assignments')
    params_.current_assignments = zeros(size(params_.mobile_chargers, 1), 1);
end
if ~isfield(params_, 'charging_queue')
    params_.charging_queue = [];
end
if ~isfield(params_, 'traceRecord') || isempty(params_.traceRecord)
    params_.traceRecord = cell(params_.UAVnum, 1);
    for uav_idx = 1:params_.UAVnum
        params_.traceRecord{uav_idx} = params_.UAV_info(uav_idx, 1:2);  % Start with initial position
    end
end

% UAV movement
while (sum(params_.SumTarget) ~= params_.UAVnum)    % When all UAV arrive at target, stop iteration
    
    % Initialize charger movement intentions for this iteration
    charger_movement_intent = cell(size(params_.mobile_chargers, 1), 1);
    
    for i = 1:params_.UAVnum
        % If one UAV arrives at target, no further planning is needed
        if (norm(round(params_.UAV_pos(i, :) * params_.N) - params_.TARGET) <= params_.stepWay * 5)
            params_.SumTarget(i) = 1;
            
            % Record the completion time for this UAV only if not already recorded
            if ~params_.hasCompleted(i)
                params_.completionTime(i) = toc(params_.startTime);
                disp(['UAV ', num2str(i), ' completed the mission in ', num2str(params_.completionTime(i)), ' seconds.']);
                params_.hasCompleted(i) = true;     % Mark as completed to avoid repetition
            end
            
        else
            params_.imgnum = params_.imgnum + 1;
            
            % Delete outdated other UAVs locations in obstacles{i}
            if(~isempty(params_.obstacles{i}))      % If obstacles is not empty
                s = 1;
                while(s < size(params_.obstacles{i}, 1))
                    % If the element in obstacles is a UAV && the ith UAV hasn't arrived at TARGET
                    if(params_.obstacles{i}(s, 4) ~= 0 && params_.SumTarget(i) == 0)
                        params_.obstacles{i}(s, :) = [];    % Delete other UAVs from obstacles{i}
                    else
                        s = s + 1;
                    end
                end
            end
            
            % Detect new static obstacles and add to obstacles{i}
            j = 1;
            while (j <= params_.obstaclesSize)  % Search all in obstaclesUAV
                % Obstacles in obstaclesUAV not UAVi and within observe radius
                if (norm(params_.obstaclesUAV(j, 1:2) - params_.UAV_pos(i, :)) < params_.obserRad && params_.obstaclesUAV(j, 4) ~= i)
                    flag = 0;
                    for k = 1:size(params_.obstacles{i}, 1)   % Not already in obstacles
                        if((params_.obstaclesUAV(j, 1) == params_.obstacles{i}(k, 1) & params_.obstaclesUAV(j, 2) == params_.obstacles{i}(k, 2) & params_.obstaclesUAV(j, 3) == params_.obstacles{i}(k, 3) & params_.obstaclesUAV(j, 4) == params_.obstacles{i}(k, 4)) == 1)
                            flag = 1;
                        end
                    end
                    if(flag == 0)
                        params_.obstacles{i} = cat(1, params_.obstacles{i}, params_.obstaclesUAV(j, 1:4));   % Add new obstacle
                        params_.needReplan(i) = 1;    % Need to replan because of altered obstacles
                    end
                end
                j = j + 1;
            end
          
            % If a UAV arrives at TARGET, delete it as obstacles from other
            % UAV's obstacles
            if params_.SumTarget(1) == 1
                row_index = find(params_.obstacles{2}(:, 4) == 1);
                params_.obstacles{2}(row_index, :) = [];
                row_index = find(params_.obstacles{3}(:, 4) == 1);
                params_.obstacles{3}(row_index, :) = [];
            end
            
            if params_.SumTarget(2) == 1
                row_index = find(params_.obstacles{1}(:, 4) == 2);
                params_.obstacles{1}(row_index, :) = [];
                row_index = find(params_.obstacles{3}(:, 4) == 2);
                params_.obstacles{3}(row_index, :) = [];
            end
            
            if params_.SumTarget(3) == 1
                row_index = find(params_.obstacles{1}(:, 4) == 3);
                params_.obstacles{1}(row_index, :) = [];
                row_index = find(params_.obstacles{2}(:, 4) == 3);
                params_.obstacles{2}(row_index, :) = [];
            end
                
            % Re-plan and calculate PATH
            if params_.needReplan(i) == 1  % Trigger replanning if either is true
                params_.replanCount(i) = params_.replanCount(i) + 1;    % Count replanning events
                % Reset replanning flags
                params_.needReplan(i) = 0;                
                % E_matrix is UAVi's risk weight matrix based on current static obstacles
                E_matrix = getEmatrix(params_.N2, params_.obstacles{i});               
                % Invoke planning function to calculate a new PATH, considering both static and dynamic obstacles
                PATH{i} = planning(i, round(params_.UAV_pos(i, :) * params_.N), E_matrix);
            end
                        
            % Move one step - WITH CHARGING PRIORITY
            if params_.UAV_energy(i) <= params_.charging_threshold
                % --- ENERGY CRITICAL: Move toward nearest charger instead of target ---
                [charger_idx, charger_dist] = assignCharger(i, params_.UAV_pos(i, :), ...
                    params_.mobile_chargers, params_.current_assignments);
                
                charger_pos = params_.mobile_chargers(charger_idx, :);
                dis_to_charger = norm(charger_pos - params_.UAV_pos(i, :));
                
                if(dis_to_charger > params_.stepWay)
                    % Move toward charger
                    params_.UAV_pos(i, :) = params_.UAV_pos(i, :) + ...
                        (charger_pos - params_.UAV_pos(i, :)) * params_.stepWay / dis_to_charger;
                else
                    % Reached charger
                    params_.UAV_pos(i, :) = charger_pos;
                end
                
                % Ensure charger is assigned
                if params_.current_assignments(charger_idx) < params_.max_assignments
                    params_.current_assignments(charger_idx) = params_.current_assignments(charger_idx) + 1;
                end
                charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
                
            else
                % --- NORMAL OPERATION: Move toward mission target ---
                if(isequal(round(params_.UAV_pos(i, :) * params_.N), PATH{i}(1, 1:2)))
                    PATH{i}(1, :) = [];
                end
                tgoal = PATH{i}(1, 1:2) / params_.N;
                dis = norm(tgoal - params_.UAV_pos(i, :));
                
                if(dis > params_.stepWay)
                    params_.UAV_pos(i, :) = params_.UAV_pos(i, :) + ...
                        (tgoal - params_.UAV_pos(i, :)) * params_.stepWay / dis;
                else
                    params_.UAV_pos(i, :) = tgoal;
                end
            end
            
            % Apply charging before consuming energy for the movement
            if params_.UAV_energy(i) <= params_.charging_threshold && params_.UAV_energy(i) > 0
                [charger_idx, charger_dist] = assignCharger(i, params_.UAV_pos(i, :), ...
                    params_.mobile_chargers, params_.current_assignments);
                
                if params_.current_assignments(charger_idx) < params_.max_assignments
                    charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
                    params_.current_assignments(charger_idx) = params_.current_assignments(charger_idx) + 1;
                    
                    if charger_dist < params_.charging_radius
                        params_.UAV_energy(i) = min(params_.max_energy, params_.UAV_energy(i) + params_.charging_rate);
                        fprintf('CHARGING: UAV %d +%.1f energy at charger %d\n', i, params_.charging_rate, charger_idx);
                    end
                else
                    params_.charging_queue = [params_.charging_queue; i];
                end
                
            elseif params_.UAV_energy(i) <= 0
                % Emergency recovery for completely depleted UAVs
                [charger_idx, ~] = assignCharger(i, params_.UAV_pos(i, :), params_.mobile_chargers, params_.current_assignments);
                charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
                fprintf('EMERGENCY: UAV %d depleted. Sending charger %d\n', i, charger_idx);
            end
            
            % Calculate energy consumption for the movement and charging
            if ~isempty(params_.traceRecord{i})
                prev_pos = params_.traceRecord{i}(end,:);
                dist_moved = norm(params_.UAV_pos(i, :) - prev_pos);
            else
                dist_moved = norm(params_.UAV_pos(i, :) - params_.UAV_info(i, 1:2));
            end
            params_.UAV_energy(i) = max(0, params_.UAV_energy(i) - dist_moved * params_.energy_consumption_rate);
            
%             % Debug charging process
%             if mod(params_.iter, 5) == 0  % Print every 5 iterations
%                 fprintf('Iter %d: UAV energies: [%.1f%%, %.1f%%, %.1f%%, %.1f%%]', ...
%                     params_.iter, params_.UAV_energy(1), params_.UAV_energy(2), ...
%                     params_.UAV_energy(3), params_.UAV_energy(4));
%                 
%                 % Show charger assignments
%                 fprintf(' | Chargers: [%d, %d]', params_.current_assignments(1), params_.current_assignments(2));
%                 
%                 % Show queue length
%                 fprintf(' | Queue: %d\n', length(params_.charging_queue));
%             end
%             
%             % Prompt specific charging messages
%             if params_.UAV_energy(i) <= params_.charging_threshold
%                 fprintf('UAV %d NEEDS CHARGE: Energy=%.1f%%, Seeking charger %d\n', i, params_.UAV_energy(i), charger_idx);
%             end
%             
%             if charger_dist < params_.charging_radius
%                 fprintf('UAV %d CHARGING: +%.1f energy at charger %d (Dist: %.3f)\n', i, params_.charging_rate, charger_idx, charger_dist);
%             end
            
%             % Move one step
%             if(isequal(round(params_.UAV_pos(i, :) * params_.N), PATH{i}(1, 1:2)))
%                 PATH{i}(1, :) = [];    % If first line has been visited, delete first line
%             end
%             tgoal = PATH{i}(1, 1:2) / params_.N;        % Turn PATH from N * N size to 1 * 1 size
%             dis = norm(tgoal - params_.UAV_pos(i, :));  % Calculate distance from UAV_pos to next position
%             if(dis > params_.stepWay)
%                 % Move one step toward the tgoal direction
%                 params_.UAV_pos(i, :) = params_.UAV_pos(i, :) + (tgoal - params_.UAV_pos(i, :)) * params_.stepWay / dis;
%             else
%                 % Move to tgoal directly
%                 params_.UAV_pos(i, :) = tgoal;
%             end
            
%             % Energy consumption and charging
%             if ~isempty(params_.traceRecord{i})
%                 prev_pos = params_.traceRecord{i}(end,:);
%                 dist_moved = norm(params_.UAV_pos(i, :) - prev_pos);
%             else
%                 % First movement - use initial position from UAV_info
%                 dist_moved = norm(params_.UAV_pos(i, :) - params_.UAV_info(i, 1:2));
%             end
%             params_.UAV_energy(i) = max(0, params_.UAV_energy(i) - dist_moved * params_.energy_consumption_rate);
             
            % DEBUG: Monitor critical energy levels
            if params_.UAV_energy(i) < params_.charging_threshold + 0.1
                fprintf('Iter %d: UAV %d energy=%.2f%%, pos=[%.2f,%.2f]\n', ...
                    params_.iter, i, params_.UAV_energy(i), params_.UAV_pos(i, 1), params_.UAV_pos(i, 2));
            end
            
            % Mobile charging logic with capacity constraints
            if params_.UAV_energy(i) <= params_.charging_threshold && params_.UAV_energy(i) > 0
                % Normal charging procedure
                [charger_idx, charger_dist] = assignCharger(i, params_.UAV_pos(i, :), ...
                    params_.mobile_chargers, params_.current_assignments);
                
                if params_.current_assignments(charger_idx) < params_.max_assignments
                    charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
                    params_.current_assignments(charger_idx) = params_.current_assignments(charger_idx) + 1;
                    
                    if charger_dist < params_.charging_radius
                        params_.UAV_energy(i) = min(params_.max_energy, ...
                            params_.UAV_energy(i) + params_.charging_rate);
                    end
                else
                    params_.charging_queue = [params_.charging_queue; i];
                end
                
            elseif params_.UAV_energy(i) <= 0
                % Emergency recovery for completely depleted UAVs
                [charger_idx, ~] = assignCharger(i, params_.UAV_pos(i, :), ...
                    params_.mobile_chargers, params_.current_assignments);
                
                charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
                fprintf('EMERGENCY: UAV %d depleted. Sending charger %d\n', i, charger_idx);
            end
            
%             % Energy consumption and charging
%             if ~isempty(params_.traceRecord{i})
%                 prev_pos = params_.traceRecord{i}(end,:);
%                 dist_moved = norm(params_.UAV_pos(i, :) - prev_pos);
%             else
%                 % First movement - use initial position from UAV_info
%                 dist_moved = norm(params_.UAV_pos(i, :) - params_.UAV_info(i, 1:2));
%             end
%             params_.UAV_energy(i) = max(0, params_.UAV_energy(i) - dist_moved * params_.energy_consumption_rate);
% 
%             % Mobile charging logic with capacity constraints
%             if params_.UAV_energy(i) < params_.charging_threshold               
%                 [charger_idx, charger_dist] = assignCharger(i, params_.UAV_pos(i, :), ...
%                     params_.mobile_chargers, params_.current_assignments);
%                 
%                 if params_.current_assignments(charger_idx) < params_.max_assignments
%                     
%                     % Store movement intention instead of directly updating (avoid race condition)
%                     charger_movement_intent{charger_idx} = params_.UAV_pos(i, :);
%                     params_.current_assignments(charger_idx) = params_.current_assignments(charger_idx) + 1;
%                     
%                     % Charge if in range
%                     if charger_dist < params_.charging_radius
%                         params_.UAV_energy(i) = min(params_.max_energy, ...
%                             params_.UAV_energy(i) + params_.charging_rate);
%                     end
%                 else 
%                     % Add to queue if no available chargers
%                     params_.charging_queue = [params_.charging_queue; i];
%                 end
%             end
            
            % Update TUs demand matrix
            for k = 1:size(params_.TU_info, 1)
                if(norm(params_.TU_info(k, 1:2) - params_.UAV_pos(i, :)) <= params_.serviceRad && params_.TU_info(k, 4) == 0)  % Demand matix removes all demand from unserved TUs who locate within service radius
                    f = max(0.00001, params_.TU_info(k, 3) - 1);
                    for x = ceil(params_.N2 * max(1 / params_.N2, params_.TU_info(k, 1) - params_.serviceRad)):floor(params_.N2 * min(1, params_.TU_info(k, 1) + params_.serviceRad))
                        for y = ceil(params_.N2 * max(1 / params_.N2, params_.TU_info(k, 2) - params_.serviceRad)):floor(params_.N2 * min(1, params_.TU_info(k, 2) + params_.serviceRad))   % In kth TUs square range
                            if(norm([x / params_.N2 y / params_.N2] - params_.TU_info(k, 1:2)) <= params_.serviceRad)     % Circle the range
                                params_.TU_demand_matrix(x, y) = max(0, params_.TU_demand_matrix(x, y) - (1 - exp(-power(params_.TU_info(k, 3), params_.n) / (params_.TU_info(k, 3) + params_.B))) + (1 - exp(-power(f, params_.n) / (f + params_.B))));
                            end
                        end
                    end
                    params_.TU_info(k, 3) = f;
                    if(params_.TU_info(k, 3) < 0.0001)
                        params_.TU_info(k, 4) = 1;      % 1 means the TU is served
                        COUNT(k) = params_.imgnum;      % Record service arrival time
                    end
                end
            end
            
            % Update UAVi's new position in obstaclesUAV and traceRecord record new position
            [x3, y3] = find(params_.obstaclesUAV(:, 4) == i);
            params_.obstaclesUAV(x3, 1:2) = params_.UAV_pos(i, :);
            params_.traceRecord{i} = cat(1, params_.traceRecord{i}, params_.UAV_pos(i, :));
        end
        
        params_.iter = params_.iter + 1;
        
        if (params_.plotFigure == 1)           
            % Draw map every 30 iterations, start from iteration number 5
            if(mod(params_.imgnum, 30) == 0 || params_.imgnum == 5)
                close all;
                drawBackground;
                savefig(['K=', num2str(params_.K), '_M=', num2str(params_.M), '_img', num2str(params_.imgnum), '.fig']);
            end
        end
    end
    
    % Update charger positions based on movement intentions (after all UAVs processed)
    for c = 1:size(params_.mobile_chargers, 1)
        if ~isempty(charger_movement_intent{c})
            params_.mobile_chargers(c, :) = moveCharger(params_.mobile_chargers(c, :), ...
                charger_movement_intent{c}, params_.charger_speed);
        end
    end
    
    % Update charger assignments and handle queue
    for c = 1:size(params_.mobile_chargers, 1)
        % Release completed charges (UAVs that moved out of range)
        nearby_uavs = find(vecnorm(params_.UAV_pos - params_.mobile_chargers(c, :), 2, 2) < params_.charging_radius);
        params_.current_assignments(c) = numel(nearby_uavs);
    
        % Process queue if charger available
        if ~isempty(params_.charging_queue) && params_.current_assignments(c) < params_.max_assignments
            queued_uav = params_.charging_queue(1);
            params_.charging_queue(1) = [];
            params_.current_assignments(c) = params_.current_assignments(c) + 1;
            
            % Immediately move charger toward queued UAV
            params_.mobile_chargers(c, :) = moveCharger(params_.mobile_chargers(c, :), ...
                params_.UAV_pos(queued_uav, :), params_.charger_speed);
        end
        
        % Return to base if idle
        if params_.current_assignments(c) == 0 && isfield(params_, 'charger_base_pos')
            params_.mobile_chargers(c, :) = params_.mobile_chargers(c, :) + ...
                (params_.charger_base_pos(c, :) - params_.mobile_chargers(c, :)) * ...
                params_.charger_speed * 0.3; % Slower return speed
        end
    end
end

% Mission completion summary
if sum(params_.SumTarget) == params_.UAVnum
    mission_time = toc(params_.startTime);
    fprintf('\nMission Complete! Total time: %.2f sec\n', mission_time);
    fprintf('Individual completion times:\n');
    for i = 1:params_.UAVnum
        status = 'Completed';
        if ~params_.hasCompleted(i)
            status = 'Failed';
        end
        fprintf('UAV %d: %.2f sec (%s)\n', i, params_.completionTime(i), status);
    end
    fprintf('Replanning counts:\n');
    disp(params_.replanCount');
    
    % Display charging statistics
    fprintf('\nCharging Statistics:\n');
    fprintf('Final energy levels:\n');
    for i = 1:params_.UAVnum
        fprintf('UAV %d: %.1f%% energy remaining\n', i, params_.UAV_energy(i));
    end
    fprintf('Charger assignments: %s\n', mat2str(params_.current_assignments'));
    fprintf('Queue length: %d\n', length(params_.charging_queue));
end

% Final map
if params_.plotFigure == 1
    close all;
    drawBackground;
    savefig(sprintf('K=%d_M=%d_img%d.fig', params_.K, params_.M, params_.imgnum));
end

% % Plot results if function exists
% if exist('plotResults', 'file')
%     plotResults(params_);
% end

