%% Process all runs with rf1
clc
outen = false; % <<<<<<<<<<<<<<<<<<<<<<<<<############# SET 'false' to bypass output gif file
matFilePaths = get_mat_dir;

if outen 
    output_name = '/home/sm/Documents/MATLAB/cbfrl/cbfrl_ddmpc_test/outputs/gifs/plot_sweep.gif';
    if exist(output_name, "file")
        input("Warning, will append existing file, ENTER > CONTINUE | CTRL + C > CANCEL");
        delete(output_name);
    end
end

best_cbf = [];
for i = 1:size(matFilePaths,1)
    disp(matFilePaths{i});
    load(matFilePaths{i}, "all_data")
    this_obstacle = all_data_rf1(all_data);
    if outen
        exportgraphics(plot_run(this_obstacle), output_name, Append=true)
    end
    % all_obstacles = [ all_obstacles ; this_obstacle ];
    % Get CBF value that completes with min distance
    [~, ridx] = min(this_obstacle.dist);    % Find the table index for min dist
    best_cbf = [ best_cbf ; [this_obstacle.obs(ridx) , this_obstacle.cbf(ridx) , this_obstacle.dist(ridx)] ];
end
plot_best_cbf(best_cbf)

% ####################################################################################################################################
%%
plot_best_cbf(best_cbf)
% ####################################################################################################################################
%% Loop through .mat files, process with modified rf1
matFilePaths = get_mat_dir;
all_tests = [];
for i = 1:size(matFilePaths,1)
    disp(matFilePaths{i});
    load(matFilePaths{i}, "all_data")
    this_obstacle = all_data_rf1_full(all_data);
    all_tests = [ all_tests ; this_obstacle];
end; clearvars -except matFilePaths all_tests
% ####################################################################################################################################

%% Generate trajectory plots of all runs within .mat files
% Need to have the all_tests object in workspace - generated section above

clearvars -except all_tests matFilePaths
% matFilePaths = get_mat_dir;

for i = 1:size(matFilePaths,1)
    disp(matFilePaths{i});
    load(matFilePaths{i}, "all_data");
    if all_data(1).obs(3) >= 0    % set to zero normally, unless need to resume
        plot_obstacle_trjs(all_data, all_tests);
    end


end
% ####################################################################################################################################






























%% LOCAL FUNCTIONS
%% Function : get_mat_dir() - Read all .mat files in directory 
% UI will prompt for source directory

function matFilePaths = get_mat_dir()

    folderPath = uigetdir;  % Specify the folder containing the .mat files
    matFiles = dir(fullfile(folderPath, '*.mat')); % Get a list of all .mat files in the folder
    matFilePaths = cell(1, length(matFiles)); % Initialize a cell array to hold the file paths
    
    for k = 1:length(matFiles)  % Loop through each file and get its full path 
        matFilePaths{k} = fullfile(folderPath, matFiles(k).name);
    end
    
    matFilePaths = matFilePaths';

end
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
    
        % Read context of this run
        state = all_data(i).state.Data;                 
        cbf = all_data(i).cbfval;                       % Read the cbf parameter value applied for this run
        pos = state(:,1:2);
        t_sep = sqrt(sum((pos -p_obs).^2,2));
        min_sep = min(t_sep) - r_obs - r_rob;           % Calculate the minimium seperation disance on this run        
        
        end_state = state(end,1:2);                     % Get state at end of run
        end_target = [0  10];                           % End target 
        end_sep = norm(end_target-end_state);           % End seperation - use to filter conservative runs

        if min_sep > 0 && end_sep < 0.3*r_obs           % If no collision calculate the total distance travelled # modify to add r_obs to relax end target on larger obstacles
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


end


%% Plot single obstacle results - cbf value vs distance travelled

function fig = plot_run(this_obstacle)

    % Filter out cbf values > 1.5
    this_obstacle = this_obstacle(this_obstacle.cbf <= 1.0, :);


    % Create figure object
    fig = figure(Visible="off");
    
    x = this_obstacle.cbf;      % x axis is cbf values
    y = this_obstacle.dist;     % y axis is path distance
    y(isnan(y)) = -1;           % Change NaN values to negative for plotting
    idxBad = y <= 0;
    idxGood = y > 0;
    
    obs = sprintf('%.2f', this_obstacle.obs(3));    % read the object size for title
    

    scatter(x(idxBad), y(idxBad), 5, 'r', 'filled');    % plot the bad
    hold on
    scatter(x(idxGood), y(idxGood), 5, 'b', 'filled');  % plot the good

    
    % highlight min distance point
    [~, ridx] = min(this_obstacle.dist);    % Find the table index for min dist
    x = this_obstacle.cbf(ridx);            
    y = this_obstacle.dist(ridx);

    % scatter(x,y,10,'r',"filled","x");
    scatter(x,y,50, "green", "filled");
    hold off


    xlabel("CBF value")
    ylabel("Distance Travelled (m)")
    title("Obstacle Radius " + obs + "m")
    subtitle("Negative distance indicates bad run")

end






%% Reward Function 1 - output more data
% Same as rf1 but export data from all runs for further use
% Investigate modifying end seperation for larger obstacles

function all_runs = all_data_rf1_full(all_data)

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
    all_runs = array2table(NaN(num_runs, 4), 'VariableNames', {'cbf', 'minsep', 'dist', 'endsep'});
    
    for i = 1:num_runs                  % loop around all_data rows, each row tests different cbf value for the obstacle size
    
        % Read context of this run
        state = all_data(i).state.Data;                 
        cbf = all_data(i).cbfval;                       % Read the cbf parameter value applied for this run
        pos = state(:,1:2);
        t_sep = sqrt(sum((pos -p_obs).^2,2));
        min_sep = min(t_sep) - r_obs - r_rob;           % Calculate the minimium seperation disance on this run        
        
        end_state = state(end,1:2);                     % Get state at end of run
        end_target = [0  10];                           % End target 
        end_sep = norm(end_target-end_state);           % End seperation - use to filter conservative runs

        % Modified from rf1 - this will record total distance regardless
        % of collision or end sep
        distances = sqrt(sum(diff(pos).^2, 2));     % Calculate Euclidean distances between consecutive points
        total_distance = sum(distances);            % Sum the distances to get the total distance
    
        % Write results from this run to table
        all_runs.cbf(i) = cbf;
        all_runs.minsep(i) = min_sep;
        all_runs.dist(i) = total_distance;
        all_runs.endsep(i) = end_sep;

    end

    all_runs.obs = repmat(r_obs, height(all_runs), 1);


end

%% Function : plot_best_cbf() - Plot the best cbf points

function plot_best_cbf(best_cbf)

    x = best_cbf(:,1);
    y = best_cbf(:,2);
    p = polyfit(x, y, 2); % p contains the slope and intercept
    y_fit = polyval(p, x);
    figure;
    plot(x, y, '*'); % Plot original data points
    hold on;
    plot(x, y_fit, '-r', LineWidth=1.3); % Plot fitted line

    p = polyfit(x, y, 3); % p contains the slope and intercept
    y_fit = polyval(p, x);
    plot(x, y_fit, '-g', LineWidth=1.3); % Plot fitted line

    title("Best Tested CBF Values vs Obstacle Radius");
    %subtitle("CBF value step size : 0.0001")
    xlabel("Obstacle Radius (m)");
    % ylabel("CBF Value")
    legend('Best Tested CBF', '2nd Order Fit', '3rd Order Fit');
    yticks = get(gca, 'YTick'); % Get the current y-tick values
    
    % Create new labels by dividing each tick value by 10^-2 and appending the exponent
    new_labels = arrayfun(@(val) sprintf('%.2f', val/10^-2), yticks, 'UniformOutput', false);
    
    % Set the new y-tick labels
    set(gca, 'YTickLabel', new_labels);
    
    % Add y-axis label with exponent
    ylabel('CBF Value (x10^{-2})');

end

%%
function drawCircle(x, y, radius, lineWidth, color)
    % Function to draw a circle on a plot
    % 
    % Inputs:
    %   x - X-coordinate of the center of the circle
    %   y - Y-coordinate of the center of the circle
    %   radius - Radius of the circle
    %   lineWidth - Thickness of the circle's outline
    %   color - Color of the circle's outline (e.g., 'r', 'g', 'b', etc.)
    
    % Number of points to define the circle
    theta = linspace(0, 2*pi, 100);
    
    % Parametric equations for the circle
    x_circle = radius * cos(theta) + x;
    y_circle = radius * sin(theta) + y;
    
    % Plot the circle
    plot(x_circle, y_circle, 'LineWidth', lineWidth, 'Color', color);
end


%% Plot Trajectory for each different obstacle runs in sweep

function plot_obstacle_trjs(all_data, all_tests)
    r_obs = all_data(1).obs(3);
    output_name = "./obs_radius_" + sprintf('%.3f', r_obs) + ".gif";
    %r_rob = 0.25;
    %p_obs = [0.1, 4.7];    
    runs = size(all_data,1);
    disp_fleg = false;
    
    for i = 1:runs
    
        state = all_data(i).state.Data;
        cbf = all_data(i).cbfval;
        
        if false %cbf > 0.001
            if ~disp_fleg
                disp("Stopping image generation at cbf values > 1.2")
                disp_fleg = true;
            end
            continue
        end
    
        tolerance = 1e-6;
        row_idx = (abs(all_tests.cbf - cbf) < tolerance ) & (abs(all_tests.obs - r_obs) < tolerance);
        T = all_tests(row_idx,:);
    
        if size(T,1) ~= 1    
            disp(T);
            input("Multiple results found in table, taking the first row")
            T = T(row_idx,1);
        end
    
        % Get min sep
        min_sep = T.minsep;
        end_sep = T.endsep;
    
        txt_sep = sprintf('%.3f', min_sep);
    
        line_color = [0 0 1];
        if min_sep <= 0
            line_color = [1 0 0];
        end
    
        f1 = figure(Visible="off");
        x = state(:,1); y = state(:,2); w = state(:,3);
        plot(x, y, LineWidth=2, Color=line_color);
        xlabel("x-pos(m)");     ylabel("y-pos(m)");
        xlim([-6 6]);           ylim([-1 11]);
        ttxt = "MPC-CBF : Parameter Value " + cbf + "Min Seperation " + txt_sep + "m" ;
        title(ttxt);
        hold on;
        drawCircle(0.1, 4.7, r_obs, 2, 'r');                % add obstacle on plot       
        exportgraphics(f1, output_name, Append=true);       % append to gif
        dtxt = num2str(i)+" / " + num2str(runs);
        disp(dtxt);
    
    end
end






%% OLD SECTIONS



%% Loop Through .mat files and get best values for each #min sep only ##OLD
% 
% all_best_cbf = [];
% 
% for i = 1:size(matFilePaths,1)
% 
%     disp(matFilePaths{i});
% 
%     load(matFilePaths{i}, "all_data")
%     best_cbf = all_data_best_cbf(all_data);
% 
%     all_best_cbf = [ all_best_cbf ; best_cbf ];
% 
% 
% end
% 
% %% Plot best cbf minsep results #OLD
% 
% x = all_best_cbf.obs
% y = all_best_cbf.CBF
% 
% figure
% scatter(x,y, 5, "filled")
% title("MPC-CBF Minimum Seperation Parameter Values")