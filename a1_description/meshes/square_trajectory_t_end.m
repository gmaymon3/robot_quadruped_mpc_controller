function u = square_trajectory_t_end(x)
%UNTITLED2 Summary of this function goes here

z_height = 0.12; 

COM_pos = x(1:3);
v_com_vec = x(4:6);
v_desired = x(7:9);
hip_data = x(10:21);
FR_foot_pos = x(22:24);
FL_foot_pos = x(25:27);
RL_foot_pos = x(28:30);
RR_foot_pos = x(31:33);
T_stance = x(34);

FL_hip_pos = hip_data(1:3);
FR_hip_pos = hip_data(4:6);
RL_hip_pos = hip_data(7:9);
RR_hip_pos = hip_data(10:12);
y_offset = 0.09;
x_offset_front = 0; 
x_offset_rear_up = 0; 
x_offset_rear_norm = 0.02; 

% Find K_step - this value can be tuned with z_com_desired
z_com_desired = 0.3;
t_delta = 0.005; 
g = 9.81;
K_step = sqrt(z_com_desired/g);
vec_leng = 0:t_delta:T_stance;



%% FL
% Find P_foot_desired - FL
FL_hip_pos(3) = 0;
FL_p_foot_desired_end = FL_hip_pos+(T_stance/2)*v_desired+K_step*(v_com_vec-v_desired);
FL_p_foot_desired_end(2) = FL_p_foot_desired_end(2)+y_offset; 
FL_p_foot_desired_end(1) = FL_p_foot_desired_end(1)+x_offset_front;


if FL_p_foot_desired_end(1) > 0.4 && FL_p_foot_desired_end(1) < 0.6
    FL_p_foot_desired_end(3) = 0.1;
elseif FL_p_foot_desired_end(1) > 0.6 && FL_p_foot_desired_end(1) < 0.8
    FL_p_foot_desired_end(3) = 0.2;
elseif FL_p_foot_desired_end(1) > 0.8 && FL_p_foot_desired_end(1) < 1
    FL_p_foot_desired_end(3) = 0.3;
else
   FL_p_foot_desired_end(3) = 0;
end

% Create Trajectory Vectors
vec_FL_px = zeros(1,length(vec_leng));
vec_FL_py = zeros(1,length(vec_leng));
vec_FL_pz = zeros(1,length(vec_leng));
vec_FL_vx = zeros(1,length(vec_leng));
vec_FL_vy = zeros(1,length(vec_leng));
vec_FL_vz = zeros(1,length(vec_leng));
t_vec = zeros(1,length(vec_leng));

if FL_foot_pos(3)+z_height > COM_pos(3) -.1
    z_height = .01;
else
    z_height = 0.12; 
end

if FL_p_foot_desired_end(3) > FL_foot_pos(3) + .05 % if stepping on step
    [FL_waypoints, toa] = square_trajectory(FL_foot_pos, FL_p_foot_desired_end, z_height, T_stance);
elseif FL_p_foot_desired_end(3) < FL_foot_pos(3) - .05 
    [FL_waypoints, toa] = triangle_trajectory(FL_foot_pos, FL_foot_pos, z_height, T_stance);
end
    [FL_waypoints, toa] = triangle_trajectory(FL_foot_pos, FL_p_foot_desired_end, z_height, T_stance);

trajectory = waypointTrajectory(FL_waypoints,TimeOfArrival=toa,SampleRate=1000,SamplesPerFrame=1);

i = 1; 

for t = 0:t_delta:T_stance
    [pos, orientation, vel] = lookupPose(trajectory,t);
    vec_FL_px(i) = pos(1);
    vec_FL_py(i) = pos(2);
    vec_FL_pz(i) = pos(3);
    vec_FL_vx(i) = vel(1);
    vec_FL_vx(i) = vel(2);
    vec_FL_vx(i) = vel(3);
    t_vec(i) = t;
    i = i+1; 
end

%% FR
% Find P_foot_desired - FR
FR_hip_pos(3) = 0;
FR_p_foot_desired_end = FR_hip_pos+(T_stance/2)*v_desired+K_step*(v_com_vec-v_desired);
FR_p_foot_desired_end(2) = FR_p_foot_desired_end(2)-y_offset;
FR_p_foot_desired_end(1) = FR_p_foot_desired_end(1)+x_offset_front;


if FR_p_foot_desired_end(1) > 0.4 && FR_p_foot_desired_end(1) < 0.6
    FR_p_foot_desired_end(3) = 0.1;
elseif FR_p_foot_desired_end(1) > 0.6 && FR_p_foot_desired_end(1) < 0.8
    FR_p_foot_desired_end(3) = 0.2;
elseif FR_p_foot_desired_end(1) > 0.8 && FR_p_foot_desired_end(1) < 1
    FR_p_foot_desired_end(3) = 0.3;
else
   FR_p_foot_desired_end(3) = 0;
end

vec_FR_px = zeros(1,length(vec_leng));
vec_FR_py = zeros(1,length(vec_leng));
vec_FR_pz = zeros(1,length(vec_leng));
vec_FR_vx = zeros(1,length(vec_leng));
vec_FR_vy = zeros(1,length(vec_leng));
vec_FR_vz = zeros(1,length(vec_leng));

if FR_foot_pos(3)+z_height > COM_pos(3) -.1
    z_height = .01;
else
    z_height = 0.12; 
end

if FR_p_foot_desired_end(3) > FR_foot_pos(3) + .05 
    [FR_waypoints, toa] =square_trajectory(FR_foot_pos, FR_p_foot_desired_end, z_height, T_stance);
elseif FR_p_foot_desired_end(3) < FR_foot_pos(3) - .05 
    [FR_waypoints, toa] = triangle_trajectory(FR_foot_pos, FR_foot_pos, z_height, T_stance);
end
    [FR_waypoints, toa] = triangle_trajectory(FR_foot_pos, FR_p_foot_desired_end, z_height, T_stance);


trajectory = waypointTrajectory(FR_waypoints,TimeOfArrival=toa,SampleRate=1000,SamplesPerFrame=1);

i = 1; 

for t = 0:t_delta:T_stance
    [pos, orientation, vel] = lookupPose(trajectory,t);
    vec_FR_px(i) = pos(1);
    vec_FR_py(i) = pos(2);
    vec_FR_pz(i) = pos(3);
    vec_FR_vx(i) = vel(1);
    vec_FR_vx(i) = vel(2);
    vec_FR_vx(i) = vel(3);
    i = i+1; 
end

%% RR
% Find P_foot_desired - RR
RR_hip_pos(3) = 0;
RR_p_foot_desired_end = RR_hip_pos+(T_stance/2)*v_desired+K_step*(v_com_vec-v_desired);
RR_p_foot_desired_end(2) = RR_p_foot_desired_end(2)-y_offset;

if (RR_foot_pos(3) <= FL_p_foot_desired_end(3)-0.2)
    RR_p_foot_desired_end(1) = RR_p_foot_desired_end(1)+x_offset_rear_up;
else
    RR_p_foot_desired_end(1) = RR_p_foot_desired_end(1)+x_offset_rear_norm;
end

% Create Trajectory 
if RR_p_foot_desired_end(1) > 0.4 && RR_p_foot_desired_end(1) < 0.6
    RR_p_foot_desired_end(3) = 0.1;
elseif RR_p_foot_desired_end(1) > 0.6 && RR_p_foot_desired_end(1) < 0.8
    RR_p_foot_desired_end(3) = 0.2;
elseif RR_p_foot_desired_end(1) > 0.8 && RR_p_foot_desired_end(1) < 1
    RR_p_foot_desired_end(3) = 0.3;
else
   RR_p_foot_desired_end(3) = 0;
end


vec_RR_px = zeros(1,length(vec_leng));
vec_RR_py = zeros(1,length(vec_leng));
vec_RR_pz = zeros(1,length(vec_leng));
vec_RR_vx = zeros(1,length(vec_leng));
vec_RR_vy = zeros(1,length(vec_leng));
vec_RR_vz = zeros(1,length(vec_leng));

if RR_foot_pos(3)+z_height > COM_pos(3) -.1
    z_height = .01;
else
    z_height = 0.12; 
end

if RR_p_foot_desired_end(3) > RR_foot_pos(3) + .05 
    [RR_waypoints, toa] = square_trajectory(RR_foot_pos, RR_p_foot_desired_end, z_height, T_stance);
elseif RR_p_foot_desired_end(3) < RR_foot_pos(3) - .05 
    [RR_waypoints, toa] = triangle_trajectory(RR_foot_pos, RR_foot_pos, z_height, T_stance);
else
    [RR_waypoints, toa] = triangle_trajectory(RR_foot_pos, RR_p_foot_desired_end, z_height, T_stance);
end

trajectory = waypointTrajectory(RR_waypoints,TimeOfArrival=toa,SampleRate=1000,SamplesPerFrame=1);

i = 1; 

for t = 0:t_delta:T_stance
    [pos, orientation, vel] = lookupPose(trajectory,t);
    vec_RR_px(i) = pos(1);
    vec_RR_py(i) = pos(2);
    vec_RR_pz(i) = pos(3);
    vec_RR_vx(i) = vel(1);
    vec_RR_vx(i) = vel(2);
    vec_RR_vx(i) = vel(3);
    i = i+1; 
end

%% RL
% Find P_foot_desired - RL
RL_hip_pos(3) = 0;
RL_p_foot_desired_end = RL_hip_pos+(T_stance/2)*v_desired+K_step*(v_com_vec-v_desired);
RL_p_foot_desired_end(2) = RL_p_foot_desired_end(2)+y_offset;

if (RL_foot_pos(3) <= FR_p_foot_desired_end(3)-0.2)
    RL_p_foot_desired_end(1) = RL_p_foot_desired_end(1)+x_offset_rear_up;
else
    RL_p_foot_desired_end(1) = RL_p_foot_desired_end(1)+x_offset_rear_norm;
end

if RL_p_foot_desired_end(1) > 0.4 && RL_p_foot_desired_end(1) < 0.6
    RL_p_foot_desired_end(3) = 0.1;
elseif RL_p_foot_desired_end(1) > 0.6 && RL_p_foot_desired_end(1) < 0.8
    RL_p_foot_desired_end(3) = 0.2;
elseif RL_p_foot_desired_end(1) > 0.8 && RL_p_foot_desired_end(1) < 1
    RL_p_foot_desired_end(3) = 0.3;
else
   RL_p_foot_desired_end(3) = 0;
end

vec_RL_px = zeros(1,length(vec_leng));
vec_RL_py = zeros(1,length(vec_leng));
vec_RL_pz = zeros(1,length(vec_leng));
vec_RL_vx = zeros(1,length(vec_leng));
vec_RL_vy = zeros(1,length(vec_leng));
vec_RL_vz = zeros(1,length(vec_leng));

if RL_foot_pos(3)+z_height > COM_pos(3) -.1
    z_height = 0.01;
else
    z_height = 0.12; 
end

if RL_p_foot_desired_end(3) > RL_foot_pos(3) + .05 
    [RL_waypoints, toa] = square_trajectory(RL_foot_pos, RL_p_foot_desired_end, z_height, T_stance);
elseif RL_p_foot_desired_end(3) < RL_foot_pos(3) - .05 
    [RL_waypoints, toa] = triangle_trajectory(RL_foot_pos, RL_foot_pos, z_height, T_stance);
else
    [RL_waypoints, toa] = triangle_trajectory(RL_foot_pos, RL_p_foot_desired_end, z_height, T_stance);
end

trajectory = waypointTrajectory(RL_waypoints,TimeOfArrival=toa,SampleRate=1000,SamplesPerFrame=1);

i = 1; 

for t = 0:t_delta:T_stance
    [pos, orientation, vel] = lookupPose(trajectory,t);
    vec_RL_px(i) = pos(1);
    vec_RL_py(i) = pos(2);
    vec_RL_pz(i) = pos(3);
    vec_RL_vx(i) = vel(1);
    vec_RL_vx(i) = vel(2);
    vec_RL_vx(i) = vel(3);
    i = i+1; 
end


%   Detailed explanation goes here
u=[vec_FL_px,vec_FL_py,vec_FL_pz,vec_FL_vx,vec_FL_vy,vec_FL_vz,vec_RL_px,vec_RL_py,vec_RL_pz,vec_RL_vx,vec_RL_vy,vec_RL_vz,vec_FR_px,vec_FR_py,vec_FR_pz,vec_FR_vx,vec_FR_vy,vec_FR_vz,vec_RR_px,vec_RR_py,vec_RR_pz,vec_RR_vx,vec_RR_vy,vec_RR_vz,t_vec]';

end

function [waypoints, toa] = square_trajectory(foot_pos, p_foot_desired_end, z_height, T_stance)
    waypoints = [foot_pos(1),foot_pos(2),foot_pos(3);  % Initial position
             foot_pos(1),foot_pos(2),foot_pos(3)+z_height; 
             p_foot_desired_end(1),p_foot_desired_end(2),foot_pos(3)+z_height; 
             p_foot_desired_end(1),p_foot_desired_end(2),p_foot_desired_end(3);
             p_foot_desired_end(1),p_foot_desired_end(2),p_foot_desired_end(3)];   % Final position

    toa = [0,T_stance*3/8,T_stance*4/8,T_stance*7/8,T_stance];
end

function [waypoints, toa] = triangle_trajectory(foot_pos, p_foot_desired_end, z_height, T_stance)
    waypoints = [foot_pos(1),foot_pos(2),foot_pos(3);  % Initial position
             (p_foot_desired_end(1) - foot_pos(1))/2 + foot_pos(1),(p_foot_desired_end(2) - foot_pos(2))/2 + foot_pos(2),foot_pos(3)+z_height;  
             p_foot_desired_end(1),p_foot_desired_end(2),p_foot_desired_end(3)];   % Final position

    toa = [0,T_stance/2, T_stance];
end