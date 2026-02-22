function PATH = planning(i, initial_pos, E_matrix)

% Global parameters
global params_;

G_ori = params_.G;      % Store cost matrix G

% Initialize G with all elements are N ^ 2 and target 0
for j = 1:params_.UAVnum
    params_.G{j} = ones(params_.N, params_.N) * params_.N ^ 2;
    params_.G{j}(params_.TARGET(1), params_.TARGET(2)) = 0;
end

% Update G matrix
node = params_.TARGET;
for t = 1:params_.episodeNum  % While in max iteration num
    % Invoke dangerMatrix function to return weight matrix A
    A = dangerMatrix(node, E_matrix);  
    params_.G{i} = min(params_.G{i}, params_.G{i}(node(1), node(2)) + A);   % Reward matrix to update G to find most minimum element
    node = randi(params_.N, 1, 2);      % Randomly find 1 * 2 int matrix range(0, N)
end

% Calculate PATH by W
step = 1;
s = initial_pos;
PATH = [];
G_tmp = params_.G{i};
while(~isequal(s, params_.TARGET) && step < params_.N * 2)  % Continue to planning as long as UAV hasn't arrived at target or step < 2N
    step = step + 1;
    
    % Include cost for obstacles
    cost_add = dangerMatrix(s, E_matrix);
    
    W2 = G_tmp + cost_add;
    [row_index, column_index] = find(W2 == min(min(W2))); % Find min element in W2
    
    % There might be 1+ minimum, so use row_index(1)
    row_index = row_index(1);
    column_index = column_index(1);
    
    s = [row_index, column_index];
    G_tmp(row_index, column_index) = params_.N ^ 2;
    
    % Prevent deadlock in PATH
    if(~isempty(PATH) && ismember(s, PATH, 'rows'))
        continue;
    else
        PATH = cat(1, PATH, s);
    end
    
end

params_.G = G_ori;      % Restore G

end