function [P_total, P_hover, P_profile_hover, P_induced_hover, P_parasite] = energyModel(V_mps)
% ENERGYMODEL  Compute propulsion power (W) for a rotorcraft at speed V (m/s).
%
%   [P_total, P_hover, P_profile_hover, P_induced_hover, P_parasite] =
%       energyModel(V, params)
%
% Inputs:
%   V_mps  - horizontal speed in m/s
%   params - struct containing physical parameters (see main.m for defaults)
%
% Outputs:
%   P_total            - total propulsion power at speed V (W)
%   P_hover            - hover power (W) (power at V = 0 using profile+induced)
%   P_profile_hover    - profile (blade) component at hover (W)
%   P_induced_hover    - induced component at hover (W)
%   P_parasite         - parasite (fuselage) power at the given speed (W)
%
% Model form (implemented to follow the paper you provided):
%   P_total = P_profile(V) + P_induced(V) + P_parasite(V)
%
% Notes/assumptions:
%  - P_profile(V) is approximated from P_profile_hover with a quadratic
%    correction factor: P_profile(V) = P_profile_hover * (1 + c_profile * V^2)
%  - P_induced(V) follows a common induced-power correction:
%         P_induced(V) = P_induced_hover * ( sqrt(1 + (V^4)/(4*v0^4)) - V^2/(2*v0^2) )
%    where v0 is an average induced velocity (given in your table).
%  - P_parasite is modeled as 0.5 * rho * Cd_fuselage * Afuselage * V^3
%    (power to overcome parasitic drag ~ V^3)
%
% These forms reproduce the behavior in your Word doc (hover > cruise).
%
% Author: adapted to fit GRLA code (2025)

% Global parameters
global params_;

% Extract parameters
rho = params_.rho;                            % air density (kg/m^3)
Afuse = params_.fuselage_area;                % fuselage equivalent area (m^2)
Cd_fuse = params_.Cd_fuselage;                % drag coefficient of fuselage
v0 = params_.v_induced_avg;                   % average induced velocity (m/s)
P_profile_hover = params_.P_profile_hover;    % given in table (W)
P_induced_hover = params_.P_induced_hover;    % given in table (W)
c_profile = params_.profile_coeff;            % incremental correction factor
% parasite coefficient uses standard aero formula

% Profile power at speed V (quadratic correction)
P_profile = P_profile_hover * (1 + c_profile * V_mps^2);

% Induced power at speed V
% formula: P_i(V) = P_i_hover * ( sqrt(1 + (V^4) / (4 * v0^4)) - V^2 / (2 * v0^2) )
% Ensure v0 > 0 to avoid divide by zero
if v0 <= 0
    P_induced = P_induced_hover;
else
    P_induced = P_induced_hover * ( sqrt(1 + (V_mps^4)/(4 * v0^4)) - V_mps^2/(2 * v0^2) );
end

% Parasite (fuselage) power ~ 0.5 * rho * Cd * A * V^3 * V (power = drag * V)
% Drag = 0.5 * rho * Cd * A * V^2, so power = Drag * V = 0.5 * rho * Cd * A * V^3
P_parasite = 0.5 * rho * Cd_fuse * Afuse * V_mps^3;

% Total propulsion power
P_total = P_profile + P_induced + P_parasite;

% Hover power (V=0)
P_hover = P_profile_hover + P_induced_hover; % parasite zero at V = 0

end
