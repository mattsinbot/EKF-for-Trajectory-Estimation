clc
clear all
close all

way_pts = load('smooth_way_points.txt');

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
display(['the rank of the controllability matrix is : ' num2str(rank(ctrb(A,B)))]);
p1 = zeros(2,2);
p2 = [1 0 0 0; 0 1 0 0];
p3 = zeros(4,2);
p4 = A;
p5 = zeros(2,2);

Aprime = [p1 p2;p3 p4];
Bprime = [p5;B];
p = [-2;-2.5;-3;-3.5;-4;-4.5];

K = place(Aprime,Bprime,p);
[v,d] = eig(Aprime-Bprime*K);

tspan = way_pts(:,1);
xcord = way_pts(:,2);
ycord = way_pts(:,3);

xEul = zeros(6,length(tspan));
xEul(:,1) = [0;0;xcord(1);ycord(1);0;0];
control_val = zeros(2,length(tspan));
trim_control_val = control_val;

control_saturate = @(x)[sign(x).*min(abs(x),5)];
control = @(t,x,i)[-K*(x-[0;0;xcord(i+1);ycord(i+1);0;0])];

for i = 1:length(tspan)-1
    h = tspan(i+1)-tspan(i);
    control_val(:,i+1) = control(tspan(i+1),xEul(:,i),i);
    trim_control_val(:,i+1) = control_saturate(control_val(:,i+1));
    xEul(:,i+1) = xEul(:,i) + h*(Aprime*xEul(:,i)+Bprime*control_saturate(control_val(:,i))-[xcord(i+1);ycord(i+1);0;0;0;0]);
end

figure;
plot(xcord,ycord,'--')
hold on
plot(xEul(3,:),xEul(4,:),'.-')
legend('actual trajectory','tracked trajectory','Location','SouthEast')
grid on
title('Robots way points')

figure;
subplot(2,1,1)
plot(tspan,trim_control_val(1,:))
subplot(2,1,2)
plot(tspan,trim_control_val(2,:))