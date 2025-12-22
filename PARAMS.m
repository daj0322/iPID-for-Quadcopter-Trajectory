classdef PARAMS
    properties (Constant)

        ode_optn = odeset('RelTol', 1e-8, 'AbsTol',1e-8);


        K_opt = [1, 0.9, 0.2, 0.005, ...
            1, 0.4, 0.07, 0, ...
            6, 0.3, 0.1, 0.0052, ...
            3, 0.24, 0.1, 0]*1;  % changed ...*1 (can choose any variable)

        Mquad = 3; % mass of quadcopter in kg

        % nominal values
        L_nom = 1;
        mload_nom = 0.6;

%         t_vec = linspace(0,10,101);
        t_vec = linspace(0,15,151);

        t_final_s = 0;

        n = 2;
        m_dev = 0.4;
        L_dev = 0.8;
        samples_MC = 100;


        %%% Plotting %%%
        colorbar_min = 6;
        colorbar_max = 11; % 20 or 60

        choice_Trajectory = 3; % 1: only x-direction;  2: only y-direction;  other: both

        % only use in when:   choice_Trajectory = 3  &  choice_example = 1 (zig-zag) ... would work for the other combinations too... not intended
        choice_Shapley = 4; % 1: only 1. phase;  2: only 2. phase ; 3. only 3. phase; other: all
    end
end


