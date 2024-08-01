%% Plot All Trajectories

close all

if exist("plt.gif", "file") % check if the output file is already there.
    input("Warning - plot file exists, will be appended. Ctrl + C to cancel.")
end

for i = 1:size(all_data,1)

    state = all_data(i).state.Data;
    cbf = string(all_data(i).cbfval);
    %t = all_data(i).tout;
    
    f1 = figure(Visible="off");
    
    x = state(:,1);
    y = state(:,2);
    w = state(:,3);
    
    plot(x,y, LineWidth=2);
    xlabel("x-pos(m)");
    ylabel("y-pos(m)");
    
    xlim([-6 6]);
    ylim([-1 11]);
    
    ttxt = "MPC-CBF : Parameter Value " + cbf;
    title(ttxt);
    
    hold on;
    
    viscircles([0.1 4.7], 0.6);
    
    exportgraphics(f1, "plt.gif", Append=true);

    disp(i);

end




%% Check Min Seperation

p_obs = [0.1, 4.7];
r_obs = 0.6;
r_rob = 0.25;

all_sep = [];

for i = 1:size(all_data,1)

    state = all_data(i).state.Data;
    cbf = all_data(i).cbfval;

    pos = state(:,1:2);
    t_sep = sqrt(sum((pos -p_obs).^2,2));
    min_sep = min(t_sep) - r_obs - r_rob;
    this_sep = [cbf, min_sep];
    all_sep = [all_sep ; this_sep];
end





%% Plot Trajectory With Min Sep

close all

p_obs = [0.1, 4.7];
r_obs = all_data(1).obs(3);
r_rob = 0.25;

all_sep = [];

if exist("plt.gif", "file") % check if the output file is already there.
    input("Warning - plot file exists, will be appended. Ctrl + C to cancel.")
end

for i = 1:size(all_data,1)

    state = all_data(i).state.Data;
    cbf = string(all_data(i).cbfval);
    %t = all_data(i).tout;
    
    % Check min seperation point on this run
    pos = state(:,1:2);
    t_sep = sqrt(sum((pos -p_obs).^2,2));
    min_sep = min(t_sep) - r_obs - r_rob;
    this_sep = [cbf, min_sep];
    all_sep = [all_sep ; this_sep];
    txt_sep = sprintf('%.3f', min_sep);


    line_color = [0 0 1];
    if min_sep <= 0
        line_color = [1 0 0];
    end

    f1 = figure(Visible="off");
    
    x = state(:,1);
    y = state(:,2);
    w = state(:,3);
    
    plot(x, y, LineWidth=2, Color=line_color);
    xlabel("x-pos(m)");
    ylabel("y-pos(m)");
    
    xlim([-6 6]);
    ylim([-1 11]);
    
    ttxt = "MPC-CBF : Parameter Value " + cbf + "Min Seperation " + txt_sep + "m" ;
    title(ttxt);
    
    hold on;
    
    viscircles([0.1 4.7], 0.6);
    
    exportgraphics(f1, "plt.gif", Append=true);

    disp(i);

end