function u=TASK1_rotating_mpc(x)
% % TODO - MAKE CONTROLLER ONLY RUN EVERY dt
P = reshape(x(1:3), [], 1);
P_dot = reshape(x(4:6), [], 1);
Rot = eul2rotm(x(7:9));
wb = reshape(x(10:12), [], 1);
P1 = reshape(x(13:15), [], 1);
P2 = reshape(x(16:18), [], 1);
P3 = reshape(x(19:21), [], 1);
P4 = reshape(x(22:24), [], 1);


% TODO - GET MASS
m = 12;

%Q = diag([5, 5, 5, 320, 320, 57, .1, .1, .1, 1, 1, 1,0]);
Q = diag([5, 5, 5, 150, 150, 150, .1, .1, .1, .1, .1, .1,0]);
R = 0.001*eye(12);
dt = 0.05;

eul = rotm2eul(Rot); % ZYX

Rz_transpose = [cos(eul(1))  sin(eul(1)) 0;
                -sin(eul(1)) cos(eul(1)) 0;
                0            0           1];

% TODO = GET INERTIA MATRIX FOR a1?
Ib = diag([0.0168, 0.0565, 0.064]);

Iw = transpose(Rz_transpose)*Ib*Rz_transpose;

w_world = Rot*wb;
   
r1 = (P1 - P);
r2 = (P2 - P);
r3 = (P3 - P);
r4 = (P4 - P);



A = [zeros(3, 3) zeros(3, 3) Rz_transpose zeros(3, 3) zeros(3, 1);
     zeros(3, 3) zeros(3, 3) zeros(3, 3)   eye(3)      zeros(3, 1);
     zeros(3, 3) zeros(3, 3) zeros(3, 3)   zeros(3, 3) zeros(3, 1);
     zeros(3, 3) zeros(3, 3) zeros(3, 3)   zeros(3, 3) [0; 0; 1];
     zeros(1, 3) zeros(1, 3) zeros(1, 3)   zeros(1, 3) 0];

B = [zeros(3, 3)   zeros(3, 3)   zeros(3, 3)   zeros(3, 3);
     zeros(3, 3)   zeros(3, 3)   zeros(3, 3)   zeros(3, 3);
     inv(Iw)*S(r1) inv(Iw)*S(r2) inv(Iw)*S(r3) inv(Iw)*S(r4);
     eye(3)/m      eye(3)/m      eye(3)/m      eye(3)/m; 
     zeros(1, 3) zeros(1, 3) zeros(1, 3)   zeros(1, 3)];



A = A*dt + eye(size(A));

B = B*dt;

% equality constraints
Aeq = [eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12) zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 13) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B            zeros(13, 12);
       zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) zeros(13, 13) -A            eye(13)       zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) zeros(13, 12) -B];

eul = flip(eul);

% TODO - FIGURE OUT IF EUL IS XYZ OR ZYX
X_bar = [transpose(eul); P; w_world; P_dot; -9.81];

Beq = [A*X_bar; zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1); zeros(13, 1)];


H = blkdiag(Q, Q, Q, Q, Q, Q, Q, Q, Q, Q, R, R, R, R, R, R, R, R, R, R);
t = x(25);
v_d = x(27);
T_stance = x(29);
pos_d = P(2)+ 10*dt*v_d;
yaw_speed = pi/16;
yaw_d = eul(3)+10*dt*yaw_speed;
xd = [0; 0; yaw_d; 0.13; 0; 0.3; 0; 0; 0; 0; 0; 0; 0];

f = [transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    transpose(-transpose(xd)*Q);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);
    zeros(12, 1);];

% Compute Intervals for this set of horizons
intervals = 0:T_stance:10;

times = [t, t+dt, t+2*dt, t+ 3*dt, t+ 4*dt, t+ 5*dt, t+ 6*dt, t+ 7*dt, t+ 8*dt, t+ 9*dt];

intervals_idx = discretize(times, intervals);

% Inequality constraints
con_ground = [0 0 1;
              0 0 -1;
              1 0 -.5;
              -1 0 -.5;
              0 1  -.5;
              0 -1 -.5];

con_air = [1 0 0;
           0 1 0;
           0 0 1;
           -1 0 0;
           0 -1 0;
           0 0 -1];

values_ground = [500; -10; 0; 0; 0; 0];

values_air = [0; 0; 0; 0; 0; 0];

control_constraints = [];
control_values = [];

% Compute gait for each horizon
for i=1: length(intervals_idx)
    if mod(intervals_idx(i), 2) == 0
        % foot 1 and 4 touching
        control_constraint = blkdiag(con_ground, con_air, con_air, con_ground);
        control_constraints = blkdiag(control_constraints, control_constraint);

        control_value = [values_ground; values_air; values_air; values_ground];
        control_values = [control_values; control_value];
    else
        % foot 2 and 3 touching
        % foot 1 and 4 touching
        control_constraint = blkdiag(con_air, con_ground, con_ground, con_air);
        control_constraints = blkdiag(control_constraints, control_constraint);

        control_value = [values_air; values_ground; values_ground; values_air];
        control_values = [control_values; control_value];
    end
end

% for i=1: length(intervals_idx)
%     if mod(intervals_idx(i), 2) == 0
%         % foot 1 and 4 touching
%         control_constraint = blkdiag(con_ground, con_ground, con_air, con_ground);
%         control_constraints = blkdiag(control_constraints, control_constraint);
% 
%         control_value = [values_ground; values_ground; values_air; values_ground];
%         control_values = [control_values; control_value];
%     else
%         % foot 2 and 3 touching
%         % foot 1 and 4 touching
%         control_constraint = blkdiag(con_ground, con_ground, con_ground, con_ground);
%         control_constraints = blkdiag(control_constraints, control_constraint);
% 
%         control_value = [values_ground; values_ground; values_ground; values_ground];
%         control_values = [control_values; control_value];
%     end
% end

Ac = [zeros(240, 130) control_constraints];

bc = control_values;

% con = [0 0 1;
%       0 0 -1;
%       1 0 -.5;
%       -1 0 -.5;
%       0 1  -.5;
%       0 -1 -.5];
% 
% control_constraint = blkdiag(con, con, con, con);
% 
% control_constraints = blkdiag(control_constraint, control_constraint, control_constraint, control_constraint, control_constraint, control_constraint, control_constraint, control_constraint, control_constraint, control_constraint);
% 
% Ac = [zeros(240, 130) control_constraints];
% 
% bc = repmat([500; -10; 0; 0; 0; 0], 4*10, 1);

options = optimoptions('quadprog', 'Display', 'off');

u_opt = quadprog(H, f, Ac, bc, Aeq, Beq, [], [], [], options);


u_world = zeros(12, 1);

u_world(1:12) = u_opt(131:142);

% u(1:3) = transpose(Rot)*u_world(1:3);
% u(4:6) = transpose(Rot)*u_world(4:6);
% u(7:9) = transpose(Rot)*u_world(7:9);
% u(10:12) = transpose(Rot)*u_world(10:12);
u(1:3) = u_world(1:3);
u(4:6) = u_world(4:6);
u(7:9) = u_world(7:9);
u(10:12) = u_world(10:12);
end

function S = S(vector)

S = [0         -vector(3) vector(2)
    vector(3)  0          -vector(1)
    -vector(2) vector(1)  0];
end