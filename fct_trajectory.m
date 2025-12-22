function [xd_vec,yd_vec,zd_vec] = fct_trajectory(t_vec,choice_example)


if choice_example == 0
    xd_vec = 2.*ones(1,length(t_vec));
    yd_vec = 4.*ones(1,length(t_vec));
    zd_vec = 10.*ones(1,length(t_vec));
else
    for k = 1:length(t_vec)
        t = t_vec(k);
        zd_vec = 5.*ones(1,length(t_vec));
        if PARAMS.choice_Trajectory == 1
            if t == 0
                xd_vec(k) = 0;
                yd_vec(k) = 0;
            else
                xd_vec(k) = 5;
                yd_vec(k) = 0;
            end
        elseif PARAMS.choice_Trajectory == 2
            if t == 0
                xd_vec(k) = 0;
                yd_vec(k) = 0;
            else
                xd_vec(k) = 0;
                yd_vec(k) = 5;
            end
        else
            if t==0
                xd_vec(k) = 0;
                yd_vec(k) = 0;
            elseif t<=5
                xd_vec(k) = 3;
                yd_vec(k) = 0;
            elseif t>5 && t<=10
                xd_vec(k) = 3;
                yd_vec(k) = 3;
            else
                xd_vec(k) = 6;
                yd_vec(k) = 3;
            end
        end
    end
end
end