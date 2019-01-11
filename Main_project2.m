clc
clear all
close all
% This program combines the control and observer using seperation principle
% Controller is designed as u = -K(x - x_des)
% Full state observability is combined to the above control law


A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
C = [1 0 0 0; 0 1 0 0];
p_obs = [-18;-19;-20;-21];

L_t = place(A',C',p_obs);
L = L_t';

%================================================
% LQR Optimal Control for computing Gain matrix
%================================================
Q = eye(4);
R = .01*eye(2);
optK = LQR_k(A,B,Q,R);
[v_cont, d_cont] = eig(A-B*optK);

%===================================================
% Control law derived from Polr Placement Technique
%===================================================
% p = [-2;-2.5;-3;-3.5];
% K = place(A,B,p);
% [v,d] = eig(A-B*K);

%===========================================
% Load the smoothed way point trajectories
%===========================================
way_pts = load('smooth_way_points.txt');
tspan = way_pts(:,1);
x_des = way_pts(:,2);
y_des = way_pts(:,3);
time = way_pts(:,1);
%============================================
% Control Law for Trajectory Tracking
%============================================
control = @(dt,x,i)[-optK*(x - [x_des(i);y_des(i);0;0] )];

%============================================
% Full State Observer Design and Control
%============================================
% Define error between desired and estimate
dt = time(2) - time(1);

e_des(:,1) = [0;0;0;0];
y(:,1) = C*e_des+[x_des(1,1);y_des(1,1)];

e_est(:,1) = [-1.1;-1;0;0];
y_est(:,1) = C*e_est+[x_des(1,1);y_des(1,1)];

for i =2:length(time)
    u=-optK*e_est(:,i-1);
    
    e_des(:,i) = e_des(:,i-1)+dt*(A*e_des(:,i-1) + B*u);
    y(:,i) = C*e_des(:,i)+[x_des(i,1);y_des(i,1)];

    e_est(:,i) = e_est(:,i-1)+dt*(A*e_est(:,i-1)+B*u +L*(y(:,i-1)-C*e_est(:,i-1)-[x_des(i,1);y_des(i,1)]));
    y_est(:,i) = C*e_est(:,i)+[x_des(i,1);y_des(i,1)];
end

plot( y_est(1,:),  y_est(2,:),'r.',y_est(1,:),  y_est(2,:),'r','linewidth',1)
%plot(y(1,:),  y(2,:),'k','linewidth',2)
figure;
plot(time, y_est(1,:), time, y_est(2,:))
title('Position (Estimate)')
ylabel('Position')
xlabel('Time')
legend('q_1','q_2', 'Location', 'southeast')
grid on
grid minor

figure;
subplot(2,1,1)
plot(time,e_des(1,:),'--',time,e_est(1,:),time,e_des(2,:),'--',time,e_est(1,:))
title('States and observer estimates')
ylabel('Position')
legend('q_1','q_1 (Estimate)','q_2','q_2 (Estimate)', 'Location', 'southeast')
subplot(2,1,2)
plot(time,e_des(3,:),'--',time,e_est(3,:),time,e_des(4,:),'--',time,e_est(4,:))
ylabel('Velocity')
xlabel('Time')
legend('q_3','q_3 (Estimate)','q_4','q_4 (Estimate)')

figure;
subplot(2,1,1)
plot(time,e_des(1,:)-e_est(1,:),time,e_des(2,:)-e_est(2,:))
ylabel('Position')
title('Error: states - observer estimates')
legend('q_1 Error','q_2 Error', 'Location', 'southeast')
subplot(2,1,2)
plot(time,e_des(3,:)-e_est(3,:),time,e_des(4,:)-e_est(4,:))
ylabel('Velocity')
xlabel('Time')
legend('q_3 Error','q_4 Error', 'Location', 'southeast')