addpath("~/matlab/com/casadi/");
%%

% obs_test_set = [0.1 0.2 0.3 0.4 0.5 1.1 1.2 1.3 1.4 1.5 ]';
% obs_test_set = [1.6 1.7 1.8 1.9 2.0]';

obs_test_set = 0.6;

obs_test_set = [0.1:0.1:2.0]';

for i = 1:size(obs_test_set,1)
    obs_radius = obs_test_set(i);

    test_params(obs_radius);

end