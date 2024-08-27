function test_params(obs_radius, test_param_set,mdl)

    if isempty(test_param_set)
        test_param_set = [0.01 :0.005: 1.00]';
    end
        
    all_data = [];
    
    % obs_radius = 1.0;

    savename = "mpc_cbf_param_sweep_obs_r" + sprintf('%.2f', obs_radius) + ".mat";


    for i = 1:size(test_param_set,1)
        garma = test_param_set(i);
        run_data = ddrob_mpc_cbf_ps(garma, obs_radius);
        all_data = [all_data ; run_data];
        disp(i);
    end

    save(savename);

end