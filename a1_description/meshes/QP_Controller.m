function [u] = QP_Controller(data)
% % TODO - MAKE CONTROLLER ONLY RUN EVERY dt


m = 12;
Ib = diag([0.0168, 0.0565, 0.064]);
g = [0;0;-9.81]; 
p = data(1:3);
r1 = data(13:15)-p; %FL
r1(3) = 0; 
r2 = data(16:18)-p; %FR
r2(3) = 0; 
r3 = data(19:21)-p; %RL
r3(3) = 0; 
r4 = data(22:24)-p; %RR
r4(3) = 0; 
dp = data(4:6);
eul = data(7:9);
R = eul2rotm(eul');
wb = data(10:12);
mu = 0.5;
S = diag([2,2,10,1,2,1]); 
S = S/1000;  

alpha = 0.01;
%alpha = 0/10000; 
%Kp_p = diag([0,0,0]); Kd_p = diag([0,0,0]); Kp_R = diag([0,0,0]); Kd_R = diag([0,0,0]);
Kp_p = diag([30,30,400]); Kd_p = diag([10,10,50]); Kp_R = diag([10,10,10]); Kd_R = diag([5,5,5]);
Kp_p = Kp_p/100000; 
Kd_p = Kd_p/100000; 

Kp_R = Kp_R/1000; 
Kd_R = Kd_p/1000; 
rpy = [0,0,0];
Rd = eul2rotm(rpy);
ypr_e = rotm2eul(Rd*R');
p_d = [0;0;0.3];
rpy_e = [ypr_e(3);ypr_e(2);ypr_e(1);];
ddp_d = Kp_p*(p_d - p) + Kd_p*(-dp);
dw_d = Kp_R*rpy_e + Kd_R*(-wb);
A = [eye(3), eye(3), eye(3), eye(3); Vec2Skewmat(r1),Vec2Skewmat(r2),Vec2Skewmat(r3),Vec2Skewmat(r4)];
b_d = [m*(ddp_d - g);
R*(Ib*dw_d + cross(wb, Ib*wb))];
H = A'*S*A + alpha*eye(12); 
f = -2*b_d'*S*A;
Aqp_block = [1 0 -mu; 
            -1 0 -mu; 
             0 1 -mu;
            0 -1 -mu; 
            0 0 1;
            0 0 -1];
Aqp = blkdiag(Aqp_block,Aqp_block,Aqp_block,Aqp_block); 
bqp_block = [0;0;0;0;500;-10];
bqp = [bqp_block;bqp_block;bqp_block;bqp_block];
options = optimoptions('quadprog','Display','off'); 
u = quadprog(H,f,Aqp,bqp,[],[],[],[],[],options);
end

