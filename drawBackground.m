% drawBackground.m
% ============================================================
% Draw environment map showing:
%   - Static obstacles (contours)
%   - Terminal Units (TUs)
%   - UAV paths
%   - Mobile chargers (NEW)
%   - Legends for UAVs and chargers
% ============================================================
 
% Global parameters
global params_;
 
TARGET2plot = params_.TARGET / params_.N;
 
figure;
hold on;
 
%% --- Draw static obstacles ---
E_matrix = getEmatrix(params_.N2, params_.obstaclesUAV2plot);
[X, Y] = meshgrid(linspace(0, 1, params_.N2), linspace(0, 1, params_.N2));
contour(X, Y, E_matrix', 'LineColor', 'k', 'LineStyle', '-'); % Black solid lines for static obstacles
 
%% --- Draw Terminal Units (TUs / users) ---
for i = 1:size(params_.TU_info, 1)
    plot(params_.TU_info(i, 1), params_.TU_info(i, 2), 'ro', 'LineWidth', 1.5, ...
        'MarkerSize', 4 + 2 * params_.TU_info(i, 3)); % Size proportional to demand
end
 
%% --- Optional: visualize TU demand matrix (light background) ---
if isfield(params_, 'TU_demand_matrix')
    demand_img = mat2gray(params_.TU_demand_matrix');
    hImg = imagesc(linspace(0,1,params_.N2), linspace(0,1,params_.N2), demand_img);
    set(hImg, 'AlphaData', 0.2);  % semi-transparent background
    colormap hot;
end
 
xlabel('x-axis'); ylabel('y-axis');
axis([-0.02 1.02 -0.02 1.02]);
set(gca, 'YDir', 'normal');
set(gcf, 'Position', [900 300 600 400]);
title('UAV Paths and Mobile Chargers');
 
%% --- Draw UAVs and their paths ---
uav_colors = {'r', 'g', 'b', 'm', 'c', [0.5 0.2 0.9]}; % Red, Green, Blue, etc.
legend_handles = []; legend_labels = {};
 
for i = 1:params_.UAVnum
    tr = cat(1, params_.UAV_info(i, 1:2), params_.traceRecord{i});
    trSize = size(tr, 1);
    current_color = uav_colors{min(i, length(uav_colors))};
 
    % Plot UAV path
    hUAV = plot(tr(:, 1), tr(:, 2), '-', 'Color', current_color, 'LineWidth', 2);
    legend_handles(end+1) = hUAV;
    legend_labels{end+1} = sprintf('UAV %d', i);
 
    % Plot UAV start and end positions
    plot(tr(1,1), tr(1,2), 'ko', 'MarkerFaceColor', current_color, 'MarkerSize', 6);
    plot(tr(trSize,1), tr(trSize,2), 'ks', 'MarkerFaceColor', current_color, 'MarkerSize', 6);
end
 
%% --- Draw target ---
plot(TARGET2plot(1), TARGET2plot(2), 'kx', 'LineWidth', 2, 'MarkerSize', 10);
 
%% === NEW: Draw mobile chargers ===
if isfield(params_, 'mobile_chargers') && ~isempty(params_.mobile_chargers)
    charger_positions = params_.mobile_chargers;
    num_chargers = size(charger_positions, 1);
 
    % Pick distinct charger colors (different from UAVs)
    charger_colors = lines(num_chargers);  % built-in colormap
 
    for c = 1:num_chargers
        hChg = plot(charger_positions(c,1), charger_positions(c,2), 'p', ... % pentagram marker
            'MarkerEdgeColor', 'k', ...
            'MarkerFaceColor', charger_colors(c,:), ...
            'MarkerSize', 12, 'LineWidth', 1.5);
        legend_handles(end+1) = hChg;
        legend_labels{end+1} = sprintf('Charger %d', c);
 
        % Draw charging radius as a dashed circle
        if isfield(params_, 'charging_radius')
            theta = linspace(0, 2*pi, 100);
            cx = charger_positions(c,1) + params_.charging_radius * cos(theta);
            cy = charger_positions(c,2) + params_.charging_radius * sin(theta);
            plot(cx, cy, '--', 'Color', charger_colors(c,:), 'LineWidth', 1);
        end
    end
end
 
%% --- Final legend ---
lgd = legend(legend_handles, legend_labels, 'Location', 'eastoutside');
set(lgd, 'Box', 'off', 'FontSize', 9);
 
grid on;
hold off;
