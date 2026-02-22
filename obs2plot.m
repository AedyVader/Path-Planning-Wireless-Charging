function [ OBSTACLES ] = obs2plot

% Data format:
% Column 1 & 2 (obstacles coordinate): x,y in range(0, 1)
% Column 3 (risk radius): r in range(0, 1)
% Column 4 (id): id is 0 indicates the enemy is obstacle not UAV

OBSTACLES = [
    0.8750    0.0500    0.0300   0
    0.4250    0.1000    0.0450   0
    0.3000    0.2000    0.0500   0
    0.4350    0.2750    0.0500   0
    0.9250    0.3000    0.0500   0
    0.6000    0.5000    0.0550   0
    0.6750    0.6500    0.0250   0
    0.8500    0.6500    0.0250   0
    0.7500    0.8000    0.0625   0
    0.5500    0.8250    0.0500   0
    0.3500    0.3500    0.0111   0
    0.2500    0.2500    0.0222   0
    0.7000    0.7000    0.0333   0
    0.4500    0.4500    0.0444   0
    0.5000    0.5000    0.0555   0
    ];

end