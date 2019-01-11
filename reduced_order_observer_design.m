clc;
close all;
clear all;

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
C = [1 0 0 0; 0 1 0 0];
P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % compute it
u = [0.5 0.5]';                           % change it

display(['Observability matrix''s rank is ' num2str(rank(obsv(A,C))) '.' ])

A_z = inv(P)*A*P;
B_z = inv(P)*B;
C_z = C*P;

p = 2;                                   % change
n = size(A,1);

A_z11 = A_z(1:p,1:p);
A_z12 = A_z(1:p,p+1:n);
A_z21 = A_z(p+1:n,1:p);
A_z22 = A_z(p+1:n,p+1:n);

B_z1 = B_z(1:p,1:p);
B_z2 = B_z(p+1:n,1:p);
K_z = [10 0; 0 15];                        % compute it

poles_sys = A_z22 - K_z*A_z12;
display(['Poles of the observer system are at'])
eig(A_z22 - K_z*A_z12)

A_w2 = A_z22 - K_z*A_z12;
A_wz1 = A_z21 - K_z*A_z11 + A_z22*K_z - K_z*A_z12*K_z;
B_wz = B_z2 - K_z*B_z1;

display(['Modified observer system is dw =  A_w2 * w + A_wz1 * z1 + B_wz u' ])

t = 0:0.001:4; 
dt = t(2) - t(1);
X(:,1) = [1;1;0;0];                      % change
y(:,1) = C*X;
Z_1(:,1) = y(:,1);
w(:,1) = [0;0];                          % change
Z_2(:,1) = K_z*Z_1(:,1)+w(:,1);

X_hat(:,1) = P*[Z_1(:,1);Z_2(:,1);];

for i = 2:length(t)
    % u = .5;
    
    X(:,i) = X(:,i-1)  +dt * (A*X(:,i-1) + B*u);
    y(:,i) = C*X(:,i) ;
    
    Z_1(:,i) = y(:,i);

    w(:,i) = w(:,i-1) + dt* ( A_w2*w(:,i-1) + A_wz1*Z_1(:,i-1) + B_wz * u) ;
    
    Z_2(:,i) = K_z*Z_1(:,i)+w(:,i);
   X_hat(:,i) = P*[Z_1(:,i);Z_2(:,i);];
end

figure;
subplot(2,1,1)
plot(t,X(1,:),'--',t,X_hat(1,:))
legend('actual', 'estimated')
title('States and observer estimates')
ylabel('Position in x')
subplot(2,1,2)
plot(t,X(3,:),'--',t,X_hat(3,:))
legend('actual', 'estimated')
ylabel('Velocity in x')
xlabel('time')

figure;
subplot(2,1,1)
plot(t,X(2,:),'--',t,X_hat(2,:))
legend('actual', 'estimated')
title('States and observer estimates')
ylabel('Position in x')
subplot(2,1,2)
plot(t,X(4,:),'--',t,X_hat(4,:))
legend('actual', 'estimated')
ylabel('Velocity in x')
xlabel('time')

figure;
subplot(2,1,1)
plot(t,X(1,:)-X_hat(1,:))
title('Error observer estimates')
ylabel('Position error in x')
subplot(2,1,2)
plot(t,X(3,:)-X_hat(3,:))
ylabel('Velocity error in x')
xlabel('time')

figure;
subplot(2,1,1)
plot(t,X(2,:)-X_hat(2,:))
title('Error observer estimates')
ylabel('Position error in y')
subplot(2,1,2)
plot(t,X(4,:)-X_hat(4,:))
ylabel('Velocity error in y')
xlabel('time')