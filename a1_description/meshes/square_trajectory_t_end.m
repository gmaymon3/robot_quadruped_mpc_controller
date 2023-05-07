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
x_offset_rear_up = 0.11; 
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


if FL_foot_pos(3)+z_height > COM_pos(3)
    z_height = COM_pos(3)-(FL_foot_pos(3)-0.03);
else
    z_height = 0.12; 
end

FL_waypoints = [FL_foot_pos(1),FL_foot_pos(2),FL_foot_pos(3);  % Initial position
             FL_foot_pos(1),FL_foot_pos(2),FL_foot_pos(3)+z_height; 
             FL_p_foot_desired_end(1),FL_p_foot_desired_end(2),FL_foot_pos(3)+z_height; 
             FL_p_foot_desired_end(1),FL_p_foot_desired_end(2),FL_p_foot_desired_end(3);
             FL_p_foot_desired_end(1),FL_p_foot_desired_end(2),FL_p_foot_desired_end(3)];   % Final position

toa = [0,T_stance*3/8,T_stance*4/8,T_stance*7/8,T_stance];

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

if FR_foot_pos(3)+z_height > COM_pos(3)
    z_height = COM_pos(3)-(FR_foot_pos(3)-0.03);
else
    z_height = 0.12; 
end

FR_waypoints = [FR_foot_pos(1),FR_foot_pos(2),FR_foot_pos(3);  % Initial position
             FR_foot_pos(1),FR_foot_pos(2),FR_foot_pos(3)+z_height; 
             FR_p_foot_desired_end(1),FR_p_foot_desired_end(2),FR_foot_pos(3)+z_height; 
             FR_p_foot_desired_end(1),FR_p_foot_desired_end(2),FR_p_foot_desired_end(3);
             FR_p_foot_desired_end(1),FR_p_foot_desired_end(2),FR_p_foot_desired_end(3)];   % Final position

toa = [0,T_stance*3/8,T_stance*4/8,T_stance*7/8,T_stance];

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

if RR_foot_pos(3)+z_height > COM_pos(3)
    z_height = COM_pos(3)-(RR_foot_pos(3)-0.03);
else
    z_height = 0.12; 
end

RR_waypoints = [RR_foot_pos(1),RR_foot_pos(2),RR_foot_pos(3);  % Initial position
             RR_foot_pos(1),RR_foot_pos(2),RR_foot_pos(3)+z_height; 
             RR_p_foot_desired_end(1),RR_p_foot_desired_end(2),RR_foot_pos(3)+z_height; 
             RR_p_foot_desired_end(1),RR_p_foot_desired_end(2),RR_p_foot_desired_end(3);
             RR_p_foot_desired_end(1),RR_p_foot_desired_end(2),RR_p_foot_desired_end(3)];   % Final position

toa = [0,T_stance*3/8,T_stance*4/8,T_stance*7/8,T_stance];

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

if RL_foot_pos(3)+z_height > COM_pos(3)
    z_height = COM_pos(3)-(RL_foot_pos(3)-0.03);
else
    z_height = 0.12; 
end

RL_waypoints = [RL_foot_pos(1),RL_foot_pos(2),RL_foot_pos(3);  % Initial position
             RL_foot_pos(1),RL_foot_pos(2),RL_foot_pos(3)+z_height; 
             RL_p_foot_desired_end(1),RL_p_foot_desired_end(2),RL_foot_pos(3)+z_height; 
             RL_p_foot_desired_end(1),RL_p_foot_desired_end(2),RL_p_foot_desired_end(3);
             RL_p_foot_desired_end(1),RL_p_foot_desired_end(2),RL_p_foot_desired_end(3)];   % Final position

toa = [0,T_stance*3/8,T_stance*4/8,T_stance*7/8,T_stance];

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