addpath("~/Documents/MATLAB/com/casadi/");
% addpath("~/matlab/com/casadi");
%% Test obstacle set
% parameter sweep set in test_params
% obs_test_set = [0.1 0.2 0.3 0.4 0.5 1.1 1.2 1.3 1.4 1.5 ]';
% obs_test_set = [1.6 1.7 1.8 1.9 2.0]';
%obs_test_set = 0.6;

obs_test_set = [0.1:0.1:2.0]';
test_param_set = [0.001 ; 0.05 ; [0.01 :0.005: 1.00]' ];
for i = 1:size(obs_test_set,1)
    obs_radius = obs_test_set(i);

    test_params(obs_radius, test_param_set);

end



%% Investigate cbf values around tested best

addpath("~/Documents/MATLAB/com/casadi/");

load('./outputs/best_cbfs.mat')

range_width = 0.1; 
range_n = 1000;

best_cbf = best_cbf(~(any(isnan(best_cbf),2)),:); % remove NaNs
best_cbf = best_cbf(:,1:2);

for i = 1:size(best_cbf,1)

    obs_radius = best_cbf(i,1);
    cbf_mid = best_cbf(i,2);

    params_start = max( (cbf_mid - 0.5*range_width), range_width/range_n);
    param_test_set = linspace(params_start, params_start+range_width, range_n)';

    dtxt = "Starting test " + num2str(obs_radius) + "m obstacle.  Previous best cbf : " + num2str(cbf_mid) + ". Starting with " + params_start ;
    disp(dtxt)

    test_params(obs_radius, param_test_set)

end




%% v2 scenario - increase distance to target (20m) and obstacle (10m), also increase sim time max (40s)


obs_test_set = [0.2 :0.2:3.0]';

for i = 1:size(obs_test_set,1)
    obs_radius = obs_test_set(i);

    test_params(obs_radius);

end