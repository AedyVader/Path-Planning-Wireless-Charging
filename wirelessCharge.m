function [P_received, eta_d] = wirelessCharge(P_tx, d_m)
% WIRELESSCHARGE Compute received charging power (W) for ER charging
%
% [P_received, eta_d] = wirelessCharge(P_tx, d_m)
%
% Inputs:
%   P_tx - transmitter power (W)
%   d_m  - Euclidean distance between charger and UAV (meters)
%
% Outputs:
%   P_received - received DC-equivalent power (W)
%   eta_d      - charging efficiency (0 to 1)
%
% Efficiency model: eta(d) = ER_a / (ER_b * d^6 + ER_c)

global params_;

% Safety check: enforce zero outside max range
if d_m > params_.R_max
    P_received = 0;
    eta_d = 0;
    return;
end

% =========================================================================
% ELECTROMAGNETIC RESONANCE EFFICIENCY CALCULATION
% =========================================================================
% Formula: eta(d) = ER_a / (ER_b * d^6 + ER_c)
%
% The d^6 term models the rapid fall-off of EM coupling efficiency
% with distance. At close range (d < 50m), efficiency is high.
% Beyond ~200m, efficiency drops rapidly.

% Extract parameters
ER_a = params_.ER_a;
ER_b = params_.ER_b;
ER_c = params_.ER_c;

% Calculate efficiency with safety checks
denominator = ER_b * (d_m^6) + ER_c;

% Prevent division by zero (should never happen with proper ER_c)
if denominator < 1e-12
    warning('wirelessCharge: denominator too small (%.2e), setting eta=0', denominator);
    eta_d = 0;
    P_received = 0;
    return;
end

eta_d = ER_a / denominator;

% Clamp efficiency to [0, 1] range (100% max)
eta_d = max(0, min(1, eta_d));

% =========================================================================
% POWER CALCULATION
% =========================================================================
% Basic model: P_received = P_tx * eta(d) * beta
% where beta is transmission efficiency coefficient

beta = params_.power_transmission_coeff;

% Calculate received power
P_received = beta * P_tx * eta_d;

% Apply path loss factor if enabled (advanced model)
if params_.pathloss_factor > 0
    d_ref = params_.d_ref;
    d_eps = params_.d_eps;
    pathloss_multiplier = 1 / (1 + params_.pathloss_factor * (d_m / (d_ref + d_eps)));
    P_received = P_received * pathloss_multiplier;
end

% =========================================================================
% DEBUG OUTPUT (Enable during testing)
% =========================================================================
% Uncomment this section to see charging calculations in real-time
% 
% persistent call_count;
% if isempty(call_count)
%     call_count = 0;
% end
% call_count = call_count + 1;
% 
% % Only print every 100th call to avoid spam
% if mod(call_count, 100) == 0 && P_received > 0.1
%     fprintf('[CHARGING] d=%.1fm, eta=%.6f, P_rx=%.2fW\n', ...
%         d_m, eta_d, P_received);
% end

% =========================================================================
% VALIDATION: Warn about suspicious values
% =========================================================================
if eta_d > 0.5 && d_m > 50
    warning('wirelessCharge: Unexpectedly high efficiency (%.1f%%) at distance %.1fm', ...
        eta_d * 100, d_m);
end

if P_received > P_tx
    warning('wirelessCharge: Output power (%.1fW) exceeds input (%.1fW)! Check parameters.', ...
        P_received, P_tx);
    P_received = P_tx;  % Cap at input power
end

end

