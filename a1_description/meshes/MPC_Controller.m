function [u] = MPC_Controller(data)
% % TODO - MAKE CONTROLLER ONLY RUN EVERY dt

state_num = 13; 
controller_num = 12;
N = 10; 
dt = 0.03; 
t = data(25);
m = 12;
Ib = diag([0.0168, 0.0565, 0.064]);
Q_mpc = diag([4,5,6,10,10,10,4,4,4,1,1,1]);
R_mpc = 0.0000001*eye(12);
p_d = [0;0;0.3];
rpy = [0,0,0];
xd = [p_d;(rpy)';
    zeros(3,1);
    zeros(3,1)];

g = [0;0;-9.81]; 
p = data(1:3);
r1 = data(13:15)-p; %FL
r1(3) = 0; 
r2 = data(16:18)-p; %RL
r2(3) = 0; 
r3 = data(19:21)-p; %FR
r3(3) = 0; 
r4 = data(22:24)-p; %RR
r4(3) = 0; 
dp = data(4:6);
eul = data(7:9);
R = eul2rotm(eul');
wb = data(10:12);

q0 = [p;eul(3);eul(2);eul(1);dp;wb];
yaw = eul(1);
Rz_yaw = [cos(yaw) sin(yaw) 0;
        -sin(yaw) cos(yaw) 0; 
        0 0 1];

Act = [zeros(3) zeros(3) eye(3) zeros(3) zeros(3,1); 
       zeros(3) zeros(3) zeros(3) Rz_yaw zeros(3,1); 
       zeros(3) zeros(3) zeros(3) zeros(3) [0;0;-1]; 
       zeros(4,13)];

Iw = R'*Ib*R;
D = [m*eye(3) zeros(3);
    zeros(3) Iw];

A = [eye(3), eye(3), eye(3), eye(3);
    Vec2Skewmat(r1),Vec2Skewmat(r2),Vec2Skewmat(r3),Vec2Skewmat(r4)];

Bct = [zeros(6,12);
       D\A;
       zeros(1,12)];

At = eye(state_num)+dt*Act; 
Bt = dt*Bct;


Q_mpc = blkdiag(Q_mpc,0); 
xd = [xd;9.8];
H=[];

% Build H with Q
for i = 1:N
    H = blkdiag(H,Q_mpc);
end

% Build H with R
for i = 1:N
    H = blkdiag(H,R_mpc);
end

% Build f
f_blk = -Q_mpc*xd; 
f=[];
for i = 1:N
    f = [f;f_blk];
end


f = [f;zeros(N*controller_num,1)];
Aeq_bl1 = eye(state_num*N); 
Aeq_bl2 =[];

% Find Aeq
for i = 1:N-1
    Aeq_bl2 = blkdiag(Aeq_bl2,-At);
end

Aeq_bl2 = [zeros(state_num, state_num*N);
           Aeq_bl2, zeros(state_num*(N-1),state_num)];

Aeq_bl3 =[];
for i = 1:N
    Aeq_bl3 = blkdiag(Aeq_bl3,-Bt); 
end

Aeq = [Aeq_bl1+Aeq_bl2 Aeq_bl3];
Beq = [At*[q0;9.8]; zeros((N-1)*state_num,1)];

mu = 0.5;
Aineq_blk = [1 0 -mu;
            -1 0 -mu; 
             0 1 -mu; 
             0 -1 -mu; 
             0 0 1;
            0 0 -1];

A_ineq = [];
for i = 1:(N*4)
    A_ineq = blkdiag(A_ineq,Aineq_blk);
end

A_ineq = [zeros(6*N*4,N*state_num) A_ineq];
b_ineq = [];

mpctable = gait(t,N,dt);

for i = 0:N-1 
    for j = 1:4
        b_ineq_blk = [0;0;0;0;500*mpctable(4*i + j);-10*mpctable(4*i + j)];
        b_ineq =[b_ineq;b_ineq_blk];
    end 
end

options = optimoptions('quadprog','Display','off');
X_star = quadprog(H,f,A_ineq,b_ineq,Aeq,Beq,[],[],[],options); 
u = X_star(N*state_num+1:N*state_num+controller_num);

% for z = 1:1:12
%     if u(z) > 100
%         u(z) = 0; 
%     end
% end

end

