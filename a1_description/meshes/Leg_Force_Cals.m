% q = [0;0.8;-1];
% 
% leg = 3;
% l1 = 0.04; 
% l2 = 0.2; 
% l3 = 0.2; 
% if leg==1 || leg==3 % left leg has sideSign 1
%     sideSign=1;
% else
%     sideSign=-1; % right leg has sideSign -1
% end
% s1=sin(q(1)); % for hip joint
% s2=sin(q(2)); % for thigh joint
% s3=sin(q(3)); % for calf joint
% 
% c1=cos(q(1)); % for hip joint
% c2=cos(q(2)); % for thigh joint
% c3=cos(q(3)); % for calf joint
% 
% c23=c2*c3-s2*s3;
% s23=s2*c3+c2*s3;
% 
% J(1,1)=0;
% J(2,1)=-sideSign*l1*s1+l2*c2*c1+l3*c23*c1;
% J(3,1)=sideSign*l1*c1+l2*c2*s1+l3*c23*s1;
% 
% J(1,2)=-l3*c23-l2*c2;
% J(2,2)=-l2*s2*s1-l3*s23*s1;
% J(3,2)=l2*s2*c1+l3*s23*c1;
% 
% J(1,3)=-l3*c23;
% J(2,3)=-l3*s23*s1;
% J(3,3)=l3*s23*c1;
% 
% F_RL = [0;0;1];
% inJ_R_RL = transpose(J);
% Tau_RL = -inJ_R_RL*F_RL; 
% 
% steps = [0.2;0.4;0.6;0.8;1]; 
% t = 0.6; 
% a = discretize(t,steps); 


v_desired = [0;0;3];
vx = v_desired(1); 
vy = v_desired(2); 
vz = v_desired(3); 


z_height = 0.03; 
vz = z_height/0.1; 

i = 1; 
for t = 0:0.01:0.7

half_steps = [0.1;0.3;0.5;0.7;0.9;1.1];
steps = [0;0.2;0.4;0.6;0.8;1];


if t <= 0.2; 
    pz = 0;
    px = 0; 
    py = 0; 
    
else
    step_index = discretize(t,steps);
    delta_t = t-steps(step_index);
   
    half_step_index = discretize(t,half_steps);
    is_odd = rem(half_step_index,2);
    % Find the desired position of the foot using the time
    px = vx*delta_t; 
    py = vy*delta_t; 
    pz = vz*delta_t; 
    if pz > z_height
        pz = z_height-vz*(delta_t-0.1)
    end

end

p_desired = [px;py;pz];
pz_all(i) = pz; 
t_all(i) = t; 
i = i+1; 

end
hold on 
plot(t_all,pz_all)
%plot(t_all,is_odd_all)
hold off 
