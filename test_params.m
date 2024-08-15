function test_params(obs_radius, test_param_set,mdl)

    if isempty(test_param_set)
        test_param_set = [0.01 :0.005: 1.00]';
    end

    if isempty(mdl)
        mdl = 1;
    else
        mdl = 2;
    end
        
        all_data = [];
    
    % obs_radius = 1.0;

    savename = "mpc_cbf_param_sweep_obs_r" + sprintf('%.2f', obs_radius) + ".mat";


    for i = 1:size(test_param_set,1)
        garma = test_param_set(i);
        if mdl == 1
            run_data = ddrob_mpc_cbf_ps(garma, obs_radius);
        else
            run_data = ddrob_mpc_cbf_ps_v2(garma, obs_radius);
        end
        all_data = [all_data ; run_data];
        disp(i);
    end

    save(savename);

end