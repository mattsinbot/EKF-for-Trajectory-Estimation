vUni = 1.5;
t = zeros(1,length(new_way));

for i = 1:length(new_way)-1
   d = norm(new_way(:,i+1)-new_way(:,i));
   del_t = d/vUni;
   t(i+1) = t(i) + del_t;
end

% print the trajectory into a text file
traj = [t; new_way(1,:);new_way(2,:)];
fileID = fopen('smooth_way_points.txt','w');
fprintf(fileID,'%6.2f %12.8f %12.8f\n',traj);
fclose(fileID);

figure;
plot(new_way(1,:), new_way(2,:),'.-')
xlabel('x')
ylabel('y')
title('Trajectory of the robot')
grid on

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0]; 
B = [0 0; 0 0; 1 0; 0 1];
p = [-2;-3;-4;-5];

K = place(A,B,p);
[v,d] = eig(A-B*K);

w1 = 2;
w2 = 3;

control = @(t,x)[-K*(x - [sin(w1*t);sin(w2*t);w1*cos(w1*t);w2*cos(w2*t)])];
sys_dyn = @(t,x)[A*x + B*control(t,x)];
Tspan = 0:0.1:10;
x0 = [0;0;0;0];

[t,x] = ode45(sys_dyn,Tspan,x0);

figure;
subplot(2,1,1)
plot(t,x(:,1),t,sin(w1*t))
legend('x_1','sin(2t)')
title(['\lambda_A are ' num2str(d(1,1)) ' , ' num2str(d(2,2)) ',' num2str(d(3,3)) ' and ' num2str(d(4,4))])
subplot(2,1,2)
plot(t,x(:,2),t,sin(w2*t))
legend('x_2','sin(3t)')

figure;
plot(t',control(t',x'))
legend('u_x','u_y')
axis([0 10 -10 10])
title('control inputs')