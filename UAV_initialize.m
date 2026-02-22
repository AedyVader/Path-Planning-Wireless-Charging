function UAV_info = UAV_initialize

% Data format: 
% Column 1 & 2 (UAVs coordinate): x,y in range(0, 1) 
% Column 3 (risk radius): r in range(0, 1)
% Column 4 (id): id is positive integer

UAV_info = [
    0.1    0.05    0.01   1
    0.3    0.05    0.01   2
    0.5    0.05    0.01   3     
    0.7    0.05    0.01   4
    0.9    0.05    0.01   5
    ];

end