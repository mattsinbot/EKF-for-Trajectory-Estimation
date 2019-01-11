clc
clear all
close all
% This program combines the control and observer using seperation principle
% Controller is designed as u = -K(x - x_des)
% Full state observability is combined to the above control law


A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
C = [1 0 0 0; 0 1 0 0];
p_obs = [-10;-11;-12;-13];

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

%============================================
% Control Law for Trajectory Tracking
%============================================
control = @(t,x,i)[-optK*(x - [x_des(i);y_des(i);(x_des(i)-x_des(i-1))/(tspan(i)-tspan(i-1));(y_des(i)-y_des(i-1))/(tspan(i)-tspan(i-1))] )];

%============================================
% Full State Observer Design and Control
%============================================
x0 = [x_des(1);y_des(1);0;0];

tt = tspan';
X_act(:,1) = x0;
Y_act(:,1) = C*X_act(:,1);
X_hat(:,1) = [0;0;0;0];
Y_hat(:,1) = C*X_hat;

for i = 2:length(tt)
    
    dt = tt(i) - tt(i-1);
    
    u(:,i) = control(tt(i),X_hat(:,i-1),i);
    
    X_act(:,i) = X_act(:,i-1) + dt*(A*X_act(:,i-1)+B*u(:,i));
    Y_act(:,i) = C*X_act(:,i);
    
    X_hat(:,i) = X_hat(:,i-1)  + dt*(A*X_hat(:,i-1) + B*u(:,i) +L*(Y_act(:,i-1)-Y_hat(:,i-1)));
    Y_hat(:,i) = C*X_hat(:,i);
    
end

figure;
subplot(2,1,1)
plot(tt,X_act(1,:),tt,x_des,'--')
ylabel('position in x')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
title('Trajectory tracking in X and Y directions')
grid on
subplot(2,1,2)
plot(tt,X_act(2,:),tt,y_des,'--')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
xlabel('time')
ylabel('position in y')
grid on

figure;
subplot(2,1,1)
plot(tt,X_act(3,:))
title('Velocities in X and Y directions')
ylabel('v_x')
subplot(2,1,2)
plot(tt,X_act(4,:))
xlabel('time')
ylabel('v_y')

figure;
plot(X_act(1,:),X_act(2,:),x_des,y_des,'--')
title('Trajectory tracking in XY plane')
xlabel('x')
ylabel('y')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
grid on

figure;
subplot(2,1,1)
plot(tt,u(1,:))
axis([0 10 -10 10])
legend('u_x')
subplot(2,1,2)
plot(tt,u(2,:))
legend('u_y')
axis([0 10 -10 10])