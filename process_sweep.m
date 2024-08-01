%% Read all .mat files in directory 
% ### CURRENT PATH MUST BE .mat DIR

clear


% Specify the folder containing the .mat files
folderPath = pwd;

% Get a list of all .mat files in the folder
matFiles = dir(fullfile(folderPath, '*.mat'));

% Initialize a cell array to hold the file paths
matFilePaths = cell(1, length(matFiles));

% Loop through each file and get its full path
for k = 1:length(matFiles)
    matFilePaths{k} = fullfile(folderPath, matFiles(k).name);
end

matFilePaths = matFilePaths';

clearvars -except matFilePaths


%% Loop Through .mat files and get best values for each #min sep only ##OLD

all_best_cbf = [];

for i = 1:size(matFilePaths,1)

    disp(matFilePaths{i});

    load(matFilePaths{i}, "all_data")
    best_cbf = all_data_best_cbf(all_data);

    all_best_cbf = [ all_best_cbf ; best_cbf ];


end

%% Plot best cbf minsep results

x = all_best_cbf.obs
y = all_best_cbf.CBF

figure
scatter(x,y, 5, "filled")
title("MPC-CBF Minimum Seperation Parameter Values")

%% Loop through .mat files, process with reward function


output_name = 'plot_sweep.gif';
if exist("output_name", "file")
    disp("Warning, will overwrite existing file, ENTER > CONTINUE | CTRL + C > CANCEL");
    delete(output_name);
end
all_obstacles = [];

for i = 1:size(matFilePaths,1)

    disp(matFilePaths{i});

    load(matFilePaths{i}, "all_data")
    this_obstacle = all_data_rf1(all_data);

    exportgraphics(plot_run(this_obstacle), output_name, Append=true)

    all_obstacles = [ all_obstacles ; this_obstacle ];


end
clearvars -except all_obstacles matFilePaths this_obstacle


%%


x = all_data_rf1(all_data);

%%




























%%

%%

%% LOCAL FUNCTIONS
%% Rank based on minimum seperation
% Not a great metric, path could still be very long with very small min sep

function best_cbf = all_data_best_cbf(all_data)

    r_rob = 0.25;
    try
        p_obs = all_data(1).obs(1:2);
        r_obs = all_data(1).obs(3);
    catch
        p_obs = [0.1, 4.7];
        r_obs = input("Need obstacle size for this one...");
    end

    all_sep = [];
    
    for i = 1:size(all_data,1)
    
        state = all_data(i).state.Data;
        cbf = all_data(i).cbfval;
    
        pos = state(:,1:2);
        t_sep = sqrt(sum((pos -p_obs).^2,2));
        min_sep = min(t_sep) - r_obs - r_rob;
        this_sep = array2table([cbf, min_sep], 'VariableNames', {'CBF', 'MinSep'});
        all_sep = [all_sep ; this_sep];
    end
    
    % Filter the table to include only rows where MinSep > 0
    filtered_table = all_sep(all_sep.MinSep > 0, :);
    
    % Sort the filtered table based on the MinSep column in ascending order
    sorted_table = sortrows(filtered_table, 'MinSep');
    
    % Select the first 5 rows of the sorted table
    best_cbf = sorted_table(1:min(20, height(sorted_table)), :);

    best_cbf.obs = repmat(r_obs, height(best_cbf), 1);

end

%% Rank based on Reward Function
% Reward function based on the total distance travelled


function all_runs = all_data_rf1(all_data)

    r_rob = 0.25;
    try
        p_obs = all_data(1).obs(1:2);
        r_obs = all_data(1).obs(3);
    catch
        p_obs = [0.1, 4.7];
        r_obs = input("Need obstacle size for this one...");
    end


    
    num_runs = size(all_data, 1);  % Number of iterations

    % Preallocate an empty table with the same structure as `this_run`
    all_runs = array2table(NaN(num_runs, 3), 'VariableNames', {'cbf', 'minsep', 'dist'});
    
    for i = 1:num_runs                  % loop around all_data rows, each row tests different cbf value for the obstacle size
    
            
        state = all_data(i).state.Data;                 
        cbf = all_data(i).cbfval;                       % Read the cbf parameter value applied for this run
        pos = state(:,1:2);
        t_sep = sqrt(sum((pos -p_obs).^2,2));
        min_sep = min(t_sep) - r_obs - r_rob;           % Calculate the minimium seperation disance on this run

        if min_sep > 0                                  % If no collision calculate the total distance travelled
            distances = sqrt(sum(diff(pos).^2, 2));     % Calculate Euclidean distances between consecutive points
            total_distance = sum(distances);            % Sum the distances to get the total distance
        else
            total_distance = NaN;                       % If there is a collision the total distance does not matter
        end
    
        % Write results from this run to table
        all_runs.cbf(i) = cbf;
        all_runs.minsep(i) = min_sep;
        all_runs.dist(i) = total_distance;

    end

    all_runs.obs = repmat(r_obs, height(all_runs), 1);

    % % Filter the table to include only rows where MinSep > 0
    % filtered_table = all_runs(all_runs.minsep > 0, :);
    % 
    % % Sort the filtered table based on the distance column in ascending order
    % sorted_table = sortrows(filtered_table, 'dist');
    % 
    % % Select the first n rows of the sorted table
    % best_cbf = sorted_table(1:min(20, height(sorted_table)), :);
    % 
    % best_cbf.obs = repmat(r_obs, height(best_cbf), 1);

end


%%

function fig = plot_run(this_obstacle)

    % Filter out cbf values > 1.5
    this_obstacle = this_obstacle(this_obstacle.cbf <= 1.5, :);

    fig = figure(Visible="off");
    
    x = this_obstacle.cbf;
    y = this_obstacle.dist;
    obs = sprintf('%.2f', this_obstacle.obs(3));
    
    y(isnan(y)) = -1;
    
    scatter(x,y, 5, "filled")
    
    xlabel("CBF value")
    ylabel("Distance Travelled (m)")
    title("Obstacle Radius " + obs + "m")
    subtitle("Negative distance indicates bad run")

end