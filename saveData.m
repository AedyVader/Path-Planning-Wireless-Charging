function saveData(pathLength_list, QoS_list, Risk_list, isSigmoid)

% Global parameters
global params_;

% Organize results
K_row = repelem(params_.k_list, length(params_.m_list));
M_row = repmat(params_.m_list, 1, length(params_.k_list));
data_ori = [K_row; M_row; pathLength_list; QoS_list; Risk_list];
[rowNum, colNum] = size(data_ori);
data_cell = mat2cell(data_ori, ones(rowNum, 1), ones(colNum, 1));
title = {'K'; 'M'; 'PathLength'; 'QoS'; 'Risk'};
results = [title, data_cell];

% Write to file
%if (isSigmoid)
 %   filename = [datestr(now, 30), '-sigmoid', '.xlsx']; % Get current time
%else
 %   filename = [datestr(now, 30), '-linear', '.xlsx'];  % Get current time
%end

% Specify the filename
filename = 'Results.xlsx';

% Write the cell to an Excel file
writecell(results, filename)

end