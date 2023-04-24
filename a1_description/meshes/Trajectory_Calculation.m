%% This should be run at the beginning of the step phase
% Find K_step - this value can be tuned with z_com_desired
clc;clear;
z_com_desired = 0.275;
g = 9.81;
K_step = sqrt(z_com_desired/g);
T_stance = 0.2; 
t_last = -T_stance; 
v_com_vec = [0;0;0];
t = 0; 
v_desired = [0;0;0]; 
p_hip = [0.1805;0.047;0]; 
p_foot = [0.1805;0.047;0];
t_last_in = 0; 
steps_total = 20; 

steps = [0];
for i = 1:1:steps_total
    steps(i) = steps(end)+T_stance;

end

for t = 0:0.1:1
    m = sum(gt(t,steps));
    is_even = rem(m, 2) == 0 
    if t == 0; 
        is_even_last = false
    end
    [v,is_even_last] = fcn(v_com_vec,t,T_stance,v_desired,p_hip,is_even_last,is_even);
end

  
% See trajectory
% i = 1; 
% px = []; 
% py = []; 
% px(i) = v(1)*t; 
% py(i) = v(2)*t;
% if t > T_stance/2
%     pz(i) = z_height-v(3)*(t-half_step_time);
% else 
%     pz(i) = v(3)*t;
% end
% i = i+1;




%% This is run each loop 


% Find F_swing
% Kp = 1; 
% Kd = 1; 

% px(i) = v(1)*t; 
% py(i) = v(2)*t;
% if t > T_stance/2
%     pz(i) = z_height-v(3)*(t-half_step_time);
% else 
%     pz(i) = v(3)*t;
% end
% F_swing = Kp*(p_foot_desired-p_foot)+Kd*(foot_vel_desired-foot_vel)
% 
% F = F_swing;



function [v,is_even_last] = fcn(v_com_vec,t,T_stance,v_desired,p_hip,is_even_last,is_even)


%% NOTE - THIS CODE IS NOT COMPLETE YET
% Execute this only once before the step
if is_even_last == is_even
    % Find K_step - this value can be tuned with z_com_desired
    z_com_desired = 0.275;
    g = 9.81;
    K_step = sqrt(z_com_desired/g);

    % Store the last t value 
    t_last_out = t;

    % Find P_foot_desired
    p_foot_desired_end = p_hip+(T_stance/2)*v_com_vec+K_step*(v_com_vec-v_desired);
    
    % Create Trajectory 
    z_height = 0.01; 
    half_step = (p_foot_desired_end - p_hip)/2+[0;0;z_height];
    half_step_time = T_stance/2;
    v = half_step/half_step_time;
    is_even_last = ~is_even; 
else
    is_even_last = is_even; 
    v = v; 
end 


% p_foot_desired = v*t; 
% foot_vel_desired = v; 
% % Find F_swing
% Kp = 1; 
% Kd = 1; 
% F_swing = Kp*(p_foot_desired-p_foot)+Kd*(foot_vel_desired-foot_vel);
% 
% F = F_swing;
% t_last_out = t_last_in;
end

