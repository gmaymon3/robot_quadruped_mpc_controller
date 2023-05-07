init_foot_pos = [0;0;0]; 
des_foot_pos = [0.1;0;0];
z_height = 0.13; 
T_stance = 0.2; 

waypoints = [init_foot_pos(1),init_foot_pos(2),init_foot_pos(3);  % Initial position
             init_foot_pos(1),init_foot_pos(2),z_height; 
             des_foot_pos(1),des_foot_pos(2),z_height; 
             des_foot_pos(1),des_foot_pos(2),des_foot_pos(3)];   % Final position

delta_t = 0.005;
toa = [0,T_stance/3,2*T_stance/3,T_stance];

trajectory = waypointTrajectory(waypoints,TimeOfArrival=toa,SampleRate=1000,SamplesPerFrame=1);

[pos, orientation, vel] = lookupPose(trajectory,0.1);

figure(1)
i = 1; 
x_positions = [];
z_velos = [];

hold on 
for t = 0:delta_t:T_stance
    [pos, orientation, vel] = lookupPose(trajectory,t);
    plot(pos(1),pos(3),"b*");
    z_velos(i) = vel(3);
    x_positions(i) = pos(1);
    i = i+1; 
end

title("Position")
%axis([-1,2,-1,2])
axis square
xlabel("X")
ylabel("Z")
grid on
hold on