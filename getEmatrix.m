% getEmatrix.m
% E_matrix is a risk weight matrix (toolbox-free implementation)

function E_matrix = getEmatrix(N2, obstaclesUAV2plot)

% Input:
%   N2 - grid resolution
%   obstaclesUAV2plot - [x, y, radius, id] per row
%
% Output:
%   E_matrix - N2 x N2 matrix of risk weights

eNum = size(obstaclesUAV2plot, 1);                   
E_matrix = zeros(N2, N2);       % Initial E as a N2 * N2 zeros matrix

% Precompute grid coordinates (center of each cell)
[xg, yg] = meshgrid((1:N2)/N2, (1:N2)/N2);

for i = 1:eNum
    ox = obstaclesUAV2plot(i, 1);
    oy = obstaclesUAV2plot(i, 2);
    sigma = max(1e-6, obstaclesUAV2plot(i, 3));  % use provided "radius" as scale

    % Euclidean distance from obstacle center for each grid cell
    td = sqrt( (xg - ox).^2 + (yg - oy).^2 );

    % Gaussian radial fall-off (toolbox-free replacement for gaussmf)
    te = exp( - (td.^2) / (2 * sigma^2) );

    % Combine with existing E_matrix similar to original logic
    E_matrix = 1 - (1 - E_matrix) .* (1 - te);
end

end
