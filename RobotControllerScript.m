% U = [0 0]';

% determine values at equilibrium point for linearization
q1_equ = x_0(1); 
q2_equ = x_0(3);
% q1_equ = y_des(1);
% q2_equ = y_des(2);
q1d_equ = 0;
q2d_equ = 0;
q1dd_equ = 0;
q2dd_equ = 0;
x_equ = [q1_equ q1d_equ q2_equ q2d_equ]';
y_equ = [q1_equ; q2_equ];


delta_y_des = [y_des(1); y_des(2)] - y_equ;

T1_equ = [m1*l1^2/3 + m2*l2^2/12 + m2*(l1^2 + l2^2/4 + l1*l2*cos(q2_equ))]*q1dd_equ ...
    + [m2*l2^2/3 + m2*l1*l2/2*cos(q2_equ)]*q2dd_equ - m2*l1*l2*sin(q2_equ)*q1d_equ*q2d_equ ...
    - m2*l1*l2*sin(q2_equ)*q2d_equ^2/2 + (m1*l1/2 + m2*l1)*g*cos(q1_equ) ...
    + m2*l2/2*g*cos(q1_equ+q2_equ) + c1*q1d_equ;
    
T2_equ = [m2*l2^2/3 + m2*l1*l2/2*cos(q2_equ)]*q1dd_equ + m2*l2^2*q2dd_equ/3 ...
    + m2*l1*l2*sin(q2_equ)/2*q1d_equ^2 + m2*l2/2*g*cos(q1_equ+q2_equ) + c2*q2d_equ;

T_equ = [T1_equ T2_equ]';
 
% output from symbolic jacobian
A = [                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                     1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                    (3*(2*l1*l2*g*m1*sin(q1_equ) + 4*l1*l2*g*m2*sin(q1_equ) - 3*l1*l2*g*m2*sin(q1_equ + q2_equ)*cos(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2),                                                                                                                                                        (3*(4*l1*l2^2*m2*q1d_equ*sin(q2_equ) - 4*l2*c1 + 4*l1*l2^2*m2*q2d_equ*sin(q2_equ) + 6*l1^2*l2*m2*q1d_equ*cos(q2_equ)*sin(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (3*(6*l1*T2_equ*sin(q2_equ) - 6*l1*c2*q2d_equ*sin(q2_equ) + 2*l1*l2^2*m2*q1d_equ^2*cos(q2_equ) + 2*l1*l2^2*m2*q2d_equ^2*cos(q2_equ) + 3*l1^2*l2*m2*q1d_equ^2*cos(q2_equ)^2 - 3*l1^2*l2*m2*q1d_equ^2*sin(q2_equ)^2 - 3*l1*l2*g*m2*cos(q1_equ + q2_equ)*sin(q2_equ) - 3*l1*l2*g*m2*sin(q1_equ + q2_equ)*cos(q2_equ) + 4*l1*l2^2*m2*q1d_equ*q2d_equ*cos(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2) - (54*l1^2*l2*m2*cos(q2_equ)*sin(q2_equ)*(4*l2*T1_equ - 4*l2*T2_equ - 4*l2*c1*q1d_equ + 4*l2*c2*q2d_equ - 6*l1*T2_equ*cos(q2_equ) + 6*l1*c2*q2d_equ*cos(q2_equ) + 2*l1*l2^2*m2*q1d_equ^2*sin(q2_equ) + 2*l1*l2^2*m2*q2d_equ^2*sin(q2_equ) - 2*l1*l2*g*m1*cos(q1_equ) - 4*l1*l2*g*m2*cos(q1_equ) + 3*l1*l2*g*m2*cos(q1_equ + q2_equ)*cos(q2_equ) + 3*l1^2*l2*m2*q1d_equ^2*cos(q2_equ)*sin(q2_equ) + 4*l1*l2^2*m2*q1d_equ*q2d_equ*sin(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2)^2,                                                                                                                                            (3*(4*l2*c2 + 6*l1*c2*cos(q2_equ) + 4*l1*l2^2*m2*q1d_equ*sin(q2_equ) + 4*l1*l2^2*m2*q2d_equ*sin(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2);
                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                        1;
 -(3*(4*l1*l2^2*g*m2^2*sin(q1_equ) - 6*l1^2*l2*g*m2^2*sin(q1_equ + q2_equ) - 3*l1*l2^2*g*m2^2*sin(q1_equ + q2_equ)*cos(q2_equ) + 6*l1^2*l2*g*m2^2*cos(q2_equ)*sin(q1_equ) - 2*l1^2*l2*g*m1*m2*sin(q1_equ + q2_equ) + 2*l1*l2^2*g*m1*m2*sin(q1_equ) + 3*l1^2*l2*g*m1*m2*cos(q2_equ)*sin(q1_equ)))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2), -(3*(4*l1*l2^3*m2^2*q1d_equ*sin(q2_equ) - 4*l2^2*c1*m2 + 12*l1^3*l2*m2^2*q1d_equ*sin(q2_equ) + 4*l1*l2^3*m2^2*q2d_equ*sin(q2_equ) - 6*l1*l2*c1*m2*cos(q2_equ) + 4*l1^3*l2*m1*m2*q1d_equ*sin(q2_equ) + 12*l1^2*l2^2*m2^2*q1d_equ*cos(q2_equ)*sin(q2_equ) + 6*l1^2*l2^2*m2^2*q2d_equ*cos(q2_equ)*sin(q2_equ)))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2), (54*l1^2*l2^2*m2^2*cos(q2_equ)*sin(q2_equ)*(4*l2^2*T1_equ*m2 - 12*l1^2*T2_equ*m2 - 4*l1^2*T2_equ*m1 - 4*l2^2*T2_equ*m2 + 4*l1^2*c2*m1*q2d_equ - 4*l2^2*c1*m2*q1d_equ + 12*l1^2*c2*m2*q2d_equ + 4*l2^2*c2*m2*q2d_equ + 6*l1^2*l2*g*m2^2*cos(q1_equ + q2_equ) - 4*l1*l2^2*g*m2^2*cos(q1_equ) + 6*l1*l2*T1_equ*m2*cos(q2_equ) - 12*l1*l2*T2_equ*m2*cos(q2_equ) + 2*l1*l2^3*m2^2*q1d_equ^2*sin(q2_equ) + 6*l1^3*l2*m2^2*q1d_equ^2*sin(q2_equ) + 2*l1*l2^3*m2^2*q2d_equ^2*sin(q2_equ) + 3*l1*l2^2*g*m2^2*cos(q1_equ + q2_equ)*cos(q2_equ) + 6*l1^2*l2^2*m2^2*q1d_equ^2*cos(q2_equ)*sin(q2_equ) + 3*l1^2*l2^2*m2^2*q2d_equ^2*cos(q2_equ)*sin(q2_equ) - 6*l1*l2*c1*m2*q1d_equ*cos(q2_equ) + 12*l1*l2*c2*m2*q2d_equ*cos(q2_equ) - 6*l1^2*l2*g*m2^2*cos(q1_equ)*cos(q2_equ) + 2*l1^2*l2*g*m1*m2*cos(q1_equ + q2_equ) - 2*l1*l2^2*g*m1*m2*cos(q1_equ) + 2*l1^3*l2*m1*m2*q1d_equ^2*sin(q2_equ) + 4*l1*l2^3*m2^2*q1d_equ*q2d_equ*sin(q2_equ) + 6*l1^2*l2^2*m2^2*q1d_equ*q2d_equ*cos(q2_equ)*sin(q2_equ) - 3*l1^2*l2*g*m1*m2*cos(q1_equ)*cos(q2_equ)))/(- 9*l1^2*l2^2*m2^2*cos(q2_equ)^2 + 12*l1^2*l2^2*m2^2 + 4*m1*l1^2*l2^2*m2)^2 - (3*(6*l1^2*l2^2*m2^2*q1d_equ^2*cos(q2_equ)^2 - 6*l1^2*l2*g*m2^2*sin(q1_equ + q2_equ) + 3*l1^2*l2^2*m2^2*q2d_equ^2*cos(q2_equ)^2 - 6*l1^2*l2^2*m2^2*q1d_equ^2*sin(q2_equ)^2 - 3*l1^2*l2^2*m2^2*q2d_equ^2*sin(q2_equ)^2 - 6*l1*l2*T1_equ*m2*sin(q2_equ) + 12*l1*l2*T2_equ*m2*sin(q2_equ) + 2*l1*l2^3*m2^2*q1d_equ^2*cos(q2_equ) + 6*l1^3*l2*m2^2*q1d_equ^2*cos(q2_equ) + 2*l1*l2^3*m2^2*q2d_equ^2*cos(q2_equ) - 3*l1*l2^2*g*m2^2*cos(q1_equ + q2_equ)*sin(q2_equ) - 3*l1*l2^2*g*m2^2*sin(q1_equ + q2_equ)*cos(q2_equ) + 6*l1*l2*c1*m2*q1d_equ*sin(q2_equ) - 12*l1*l2*c2*m2*q2d_equ*sin(q2_equ) + 6*l1^2*l2*g*m2^2*cos(q1_equ)*sin(q2_equ) - 2*l1^2*l2*g*m1*m2*sin(q1_equ + q2_equ) + 6*l1^2*l2^2*m2^2*q1d_equ*q2d_equ*cos(q2_equ)^2 - 6*l1^2*l2^2*m2^2*q1d_equ*q2d_equ*sin(q2_equ)^2 + 2*l1^3*l2*m1*m2*q1d_equ^2*cos(q2_equ) + 4*l1*l2^3*m2^2*q1d_equ*q2d_equ*cos(q2_equ) + 3*l1^2*l2*g*m1*m2*cos(q1_equ)*sin(q2_equ)))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2), -(3*(4*l1^2*c2*m1 + 12*l1^2*c2*m2 + 4*l2^2*c2*m2 + 4*l1*l2^3*m2^2*q1d_equ*sin(q2_equ) + 4*l1*l2^3*m2^2*q2d_equ*sin(q2_equ) + 12*l1*l2*c2*m2*cos(q2_equ) + 6*l1^2*l2^2*m2^2*q1d_equ*cos(q2_equ)*sin(q2_equ) + 6*l1^2*l2^2*m2^2*q2d_equ*cos(q2_equ)*sin(q2_equ)))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2)];
 
 
B = [                                                                                                          0,                                                                                                                                   0;
                                            (12*l2)/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2),                                                  -(3*(4*l2 + 6*l1*cos(q2_equ)))/(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(q2_equ)^2);
                                                                                                          0,                                                                                                                                   0;
 -(3*(4*m2*l2^2 + 6*l1*m2*cos(q2_equ)*l2))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2), (3*(4*l1^2*m1 + 12*l1^2*m2 + 4*l2^2*m2 + 12*l1*l2*m2*cos(q2_equ)))/(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(q2_equ)^2)];
 
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = zeros(2,2);

% TODO: add estimator
std = deg2rad(0.333);
Qe = std^2*eye(4);
Re = std^2*eye(2);

[F,P_est,eig_est] = lqr(A,B,Qe,Re);
F = F';

% determine state feedback and regulator
Aaug = [A zeros(4,2); C zeros(2,2)];
Baug = [B; zeros(2,2)];

Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
R = eye(2);
[K1,P_lqr,eig_lqr] = lqr(A,B,Q,R);
P = 1.3*[-2.0, -2.1, -2.2, -2.3, -2.4, -2.5];
K = place(Aaug, Baug, P);
K2 = K(:,5:6);

deltaT = 0.001;
deltaY = q(:,end) - y_equ;
%deltaX = qout(end,:)' - x_equ;
e = e_prev + deltaT*(deltaY - delta_y_des);

% Plant Input
U = -K1*deltaXe_prev - K2*e + T_equ;
%U = -K1*deltaX -K2*e + T_equ; 
e_prev = e;

% Estimator
d_deltaXe = (A-F*C)*deltaXe_prev + F*deltaY + B*U;
deltaXe_prev = deltaXe_prev + d_deltaXe*deltaT; 

