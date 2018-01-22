close all; clear all; clc;

g = 9.81;
m1 = 1;
m2 = 4;
L1 = 1;
L2 = 2;
c1 = 10;
c2 = 10;

% Initial Conditions
x0 = [deg2rad(90) 0 0 0]';
% Equilibrium
xe = [deg2rad(90) 0 0 0]';

syms T1 T2 q1 q1d q1dd q2 q2d q2dd;

EoM1 = T1 == [m1*L1^2/3 + m2*L2^2/12 + m2*(L1^2 + L2^2/4 + L1*L2*cos(q2))]*q1dd ...
    + [m2*L2^2/3 + m2*L1*L2/2*cos(q2)]*q2dd - m2*L1*L2*sin(q2)*q1d*q2d ...
    - m2*L1*L2*sin(q2)*q2d^2/2 + (m1*L1/2 + m2*L1)*g*cos(q1) ...
    + m2*L2/2*g*cos(q1+q2) + c1*q1d;
    
EoM2 = T2 == [m2*L2^2/3 + m2*L1*L2/2*cos(q2)]*q1dd + m2*L2^2*q2dd/3 ...
    + m2*L1*L2*sin(q2)/2*q1d^2 + m2*L2/2*g*cos(q1+q2) + c2*q2d;

% subs(EoM1, [q1 q2 q1d q2d q1dd q2dd], [0 0 0 0 0 0])
% T1 == 16677/200 && T2 == 981/25

qdd = solve([EoM1; EoM2], [q1dd; q2dd]);
Xd = [q1d; qdd.q1dd; q2d; qdd.q2dd];
Y = [q1; q2];

A = subs(jacobian(Xd, [q1, q1d, q2, q2d]), [q1; q1d; q2; q2d], xe);
B = subs(jacobian(Xd, [T1, T2]), [T1; T2; q2], [0; 0; 0]);
C = subs(jacobian(Y, [q1, q1d, q2, q2d]), [q1; q2], [pi/2; 0]);
D = subs(jacobian(Y, [T1, T2]), [T1; T2], [0; 0]);

s = tf('s');
G = double(C)*inv(s*eye(size(A)) - double(A))*double(B);
figure(1)
pzmap(G);

Xd_lin = A*[q1; q1d; q2; q2d] + B*[T1; T2];

Xd_nlin_func = matlabFunction(Xd);
Xd_lin_func = matlabFunction(Xd_lin);

[t1,x_nlin] = ode45(@(t,x) Xd_nlin_func(0, 0, x(1), x(3), x(2), x(4)), [0 10 ]', x0);
[t2,x_lin]  = ode45(@(t,x) Xd_lin_func(0, 0, x(1), x(3), x(2), x(4)), [0 10]', x0-xe);

figure(2)
plot(t1,x_nlin(:,1));
hold on;
plot(t1,x_nlin(:,3));
hold on;
plot(t2,x_lin(:,1) + xe(1));
hold on;
plot(t2,x_lin(:,3) + xe(3));
title('Two Link Pendulum');
xlabel('Time[s]');
ylabel('Angle[Rad]');
legend('q1 nonlin', 'q2 nonlin', 'q1 lin', 'q2 lin');