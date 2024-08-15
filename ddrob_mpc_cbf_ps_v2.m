function run_data = ddrob_mpc_cbf_ps_v2(garma, r_obs)

    import casadi.*
    
    T = 0.1; %[s]
    N = 5; 
    rob_diameter = 0.5; 
    ob_avoid = 2;  % 1 relax-CBF;  2 CBF;  0  BT   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % garma = 0.01;     %########## SET FROM BATCH RUN SCRIPT ############
    
    SO_init = [0.01, 10.0, r_obs];  % <============ SET OBSTACLE LOCATION

    target_goal = [0; 20 ; pi/2];   % <============ SET TARGET 

    n_SO = size(SO_init, 1);
    v_max = 1;          % m/s
    v_min = -v_max;
    w_max = pi/4;       % rad/s
    w_min = -w_max;
    
    x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
    states = [x;y;theta]; n_states = length(states);
    
    v = SX.sym('v'); omega = SX.sym('omega');
    controls = [v;omega]; n_controls = length(controls);
    rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s
    
    f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
    U = SX.sym('U',n_controls,N); % Decision variables (controls)
    P = SX.sym('P',n_states + n_states);
    w = SX.sym('w',N,1); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X = SX.sym('X',n_states,(N+1));
    
    obj = 0; % Objective function
    g = [];  % constraints vector
    
    Q = zeros(3,3);
    Q(1,1) = 7;     % x 1
    Q(2,2) = 1;     % y 5
    Q(3,3) = 0.7;   % th 0.5
    
    % Weighing matrices (controls)
    R = zeros(2,2);
    R(1,1) = 5;   % v5
    R(2,2) = 0.1;  % omega 0.5
    
    st  = X(:,1); % initial state
    g = [g;st-P(1:3)]; % initial condition constraints
    for k = 1:N
        st = X(:,k);  con = U(:,k);
        obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
        st_next = X(:,k+1);
        f_value = f(st,con);
        st_next_euler = st+ (T*f_value);
        g = [g;st_next-st_next_euler]; % compute constraints
    end
    W = 1000*eye(3); 
    obj = obj + (X(:,N+1)-P(4:6))'*W*(X(:,N+1)-P(4:6));
    
    for k = 1:N          %%%%%%  slack variables
        W2 = 0.0001;
        obj = obj  + W2*(w(k)-1)^2;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end 
    for k = 1:N
        for i = 1:n_SO
           h = (X(1:2,k)-[SO_init(i,1);SO_init(i,2)])'*(X(1:2,k)-[SO_init(i,1);SO_init(i,2)])-(rob_diameter/2 + SO_init(i,3))^2;
           h_next = (X(1:2,k+1)-[SO_init(i,1);SO_init(i,2)])'*(X(1:2,k+1)-[SO_init(i,1);SO_init(i,2)])-(rob_diameter/2 + SO_init(i,3))^2;                   
           if ob_avoid == 1
                g = [g; h_next-w(k)*(1-garma)*h];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           elseif ob_avoid == 2
                g = [g; h_next-(1-garma)*h];
           else
                g = [g ; sqrt((X(1,k)-SO_init(i,1))^2+(X(2,k)-SO_init(i,2))^2) - (rob_diameter/2 + SO_init(i,3))];
           end       
        end
    end
    % make the decision variable one column  vector
    OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1);w];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
    
    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level =0;%0,3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol =1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    
    solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
    
    args = struct;
    args.lbg(1:3*(N+1)) = 0; % equality constraints
    args.ubg(1:3*(N+1)) = 0; % equality constraints
    
    % Obstacles representeted as inequalty constraints
    args.lbg(3*(N+1)+1:length(g)) = 0;    
    args.ubg(3*(N+1)+1:length(g)) = inf;
    
    % Constraints on states
    i_pos = n_states*(N+1);
    args.lbx(1:n_states:i_pos,1) = -10;      %state x lower bound
    args.ubx(1:n_states:i_pos,1) = 10;      %state x upper bound
    args.lbx(2:n_states:i_pos,1) = -10;      %state y lower bound
    args.ubx(2:n_states:i_pos,1) = 10;      %state y upper bound
    args.lbx(3:n_states:i_pos,1) = -inf;   %state th lower bound
    args.ubx(3:n_states:i_pos,1) = inf;    %state th upper bound
    
    % Constraints on control variables
    args.lbx(i_pos+1:n_controls:i_pos+n_controls*(N+1),1) = v_min;
    args.ubx(i_pos+1:n_controls:i_pos+n_controls*(N+1),1) = v_max; 
    args.lbx(i_pos+2:n_controls:i_pos+n_controls*(N+1),1) = w_min;
    args.ubx(i_pos+2:n_controls:i_pos+n_controls*(N+1),1) = w_max;
    
    % Constrain on slack variables
    args.lbx(i_pos+n_controls*(N)+1:i_pos+n_controls*(N)+N,1) = 0;
    args.ubx(i_pos+n_controls*(N)+1:i_pos+n_controls*(N)+N,1) = inf;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %-------------------------------------------
    t0 = 0;
    x0 = [0 ; 0 ; pi/2];    % initial condition.
    xs = target_goal; % Reference posture.
    
    x_ol(:,1) = x0; % x_ol contains the history of states
    t(1) = t0;
    
    u0 = zeros(N,2);        % two control inputs for each robot
    X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables
    w0 = zeros(N,1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sim_tim = 40; % Maximum simulation time
    
    % Start MPC
    mpciter = 0;
    x_cl = [];    % Store predicted states in the closed loop
    u_cl=[];      % Store control inputs in the closed loop
    e=[];
    
    % the main simulaton loop... it works as long as the error is greater
    % than 10^-6 and the number of mpc steps is less than its maximum
    % value.
    tic
    while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
        args.p   = [x0;xs]; % set the values of the parameters vector
        % initial value of the optimization variables
        args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1);w0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        u = reshape(full(sol.x(3*(N+1)+1:3*(N+1)+2*N))',2,N)'; % get controls only from the solution
        w0 = full(sol.x(3*(N+1)+2*N+1:end));%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        e = [e;w0];
        x_cl(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
        u_cl= [u_cl ; u(1,:)];
        t(mpciter+1) = t0;
        % Apply the control and shift the solution
        [t0, x0, u0] = shift(T, t0, x0, u,f);
        x_ol(:,mpciter+2) = x0;
        X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
        % Shift trajectory to initialize the next step
        X0 = [X0(2:end,:);X0(end,:)];
        mpciter;
        mpciter = mpciter + 1;
    end
    toc
    %%
    states = x_ol';
    run_data.state.Data = states;
    run_data.cbfval = garma;
    run_data.obs = SO_init;

end
%%
% ss_error = norm((x0-xs),2)
% Simulate_MPC_CBF_PS(x_ol,x_cl,SO_init,xs,N,rob_diameter)

%% Local functions

function [t0, x0, u0] = shift(T, t0, x0, u,f)
    st = x0;
    con = u(1,:)';
    f_value = f(st,con);
    st = st + (T*f_value);
    x0 = full(st);

    t0 = t0 + T;
    u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end