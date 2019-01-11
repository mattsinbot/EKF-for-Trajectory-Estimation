clc
close all
clear all

% This program combines the control and observer using seperation principle
% Controller is designed as u = -K(x - x_des)
% Full state observability is combined to the above control law


A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
C = [1 0 0 0; 0 1 0 0];
p_obs = [-10;-11;-12;-13];

L_t = place(A',C',p_obs);
L = L_t';

p = [-2;-2.5;-3;-3.5];
K = place(A,B,p);
[v,d] = eig(A-B*K);

way_pts = load('smooth_way_points.txt');
tspan = way_pts(:,1);
x_des = way_pts(:,2);
y_des = way_pts(:,3);

w1 = 2;
w2 = 1;

control = @(t,x,i)[-K*(x - [x_des(i);y_des(i);(x_des(i)-x_des(i-1))/(tspan(i)-tspan(i-1));(y_des(i)-y_des(i-1))/(tspan(i)-tspan(i-1))] )];
Tspan = 0:0.1:25;
x0 = [x_des(1);y_des(1);0;0];

tt = tspan';
xx = zeros(4,length(tt));
xx(:,1) = [x_des(1);y_des(1);0;0];
X(:,1) = [1;1;0;0];
y(:,1) = C*xx(:,1);
X_hat(:,1) = [0;0;0;0];
y_hat(:,1) = C*X_hat;


for i = 2:length(tt)
    dt = tt(i) - tt(i-1);
    u(:,i) = control(tt(i),xx(:,i-1),i);
    xx(:,i) = xx(:,i-1) + dt*(A*xx(:,i-1)+B*u(:,i));
    y(:,i) = C*xx(:,i);

    X_hat(:,i) = X_hat(:,i-1)  +dt * (A*X_hat(:,i-1) + B*u(:,i) +L*(y(:,i-1)-y_hat(:,i-1)));
    y_hat(:,i) = C*X_hat(:,i);
    
    
end

figure;
subplot(2,1,1)
plot(tt,xx(1,:),tt,x_des,'--')
ylabel('position in x')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
title('Trajectory tracking in X and Y directions')
grid on
subplot(2,1,2)
plot(tt,xx(2,:),tt,y_des,'--')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
xlabel('time')
ylabel('position in y')
grid on

figure;
subplot(2,1,1)
plot(tt,xx(3,:))
title('Velocities in X and Y directions')
ylabel('v_x')
subplot(2,1,2)
plot(tt,xx(4,:))
xlabel('time')
ylabel('v_y')

figure;
plot(xx(1,:),xx(2,:),x_des,y_des,'--')
title('Trajectory tracking in XY plane')
xlabel('x')
ylabel('y')
legend('tracking trajectory', 'actual trajectory','Location','SouthEast')
grid on

figure;
subplot(2,1,1)
plot(tt,u(1,:))
legend('control in X')
subplot(2,1,2)
plot(tt,u(2,:))
legend('control in Y')