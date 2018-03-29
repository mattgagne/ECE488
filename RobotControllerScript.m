% U = [0 0]';

y_true = q(:,end);
y_measured = q(:,end) + measurement_std_dev.*randn(2,1);

x_endpoint_true = l1*cos(y_true(1)) + l2*cos(y_true(1) + y_true(2));
y_endpoint_true = l1*sin(y_true(1)) + l2*sin(y_true(1) + y_true(2));
xy_endpoint_true = [x_endpoint_true; y_endpoint_true];

x_endpoint = l1*cos(y_measured(1)) + l2*cos(y_measured(1) + y_measured(2));
y_endpoint = l1*sin(y_measured(1)) + l2*sin(y_measured(1) + y_measured(2));
xy_endpoint_measured = [x_endpoint; y_endpoint];

isMajorWaypoint = (mod(waypt,num_waypts+1) == 0);

if isMajorWaypoint
    iteration_threshold = 0.001;
    required_timesteps_to_wait = 0.5/0.001;
elseif mod(waypt+1,num_waypts+1) == 0
    iteration_threshold = 0.001;
    required_timesteps_to_wait = 0;

else
    iteration_threshold = 0.01;
    required_timesteps_to_wait = 0;
end


if norm(my_waypts_xy(:,waypt) - xy_endpoint_estimated) < iteration_threshold
    waited_timesteps = waited_timesteps + 1;
    
    if waited_timesteps >= required_timesteps_to_wait 
        % iterate to next waypoint
        waypt = waypt + 1
        isMajorWaypoint = (mod(waypt,num_waypts+1) == 0);
        e_prev = 0;
        deltaXe_prev = zeros(4,1);
        
        % break after meeting end condition and stop timer
        if waypt == (size(my_waypts_ang, 2) + 1)
            return;
        end
    end 
    
else
    % if outside of desired range, zero out step counter
    waited_timesteps = 0;
end

% detremine y_des
y_des = my_waypts_ang(:,waypt);

% determine values at equilibrium point for linearization
q1_equ = my_waypts_ang(1,waypt-1);
q2_equ = my_waypts_ang(2,waypt-1); 
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
Qe = 0.1^2*eye(4);
Re = measurement_std_dev^2*eye(2);

[F,P_est,eig_est] = lqr(A,B,Qe,Re);
% F = place(A', C', [-1000, -1001, -1002, -999]);
F = F';

% determine state feedback and regulator
Aaug = [A zeros(4,2); C zeros(2,2)];
Baug = [B; zeros(2,2)];

if isMajorWaypoint
    Q = [1 0 0 0 0 0;
        0 0.01 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 0.01 0 0;
        0 0 0 0 100 0;
        0 0 0 0 0 100];
    R = 0.2*eye(2);
    [K,P_lqr,eig_lqr] = lqr(Aaug,Baug,Q,R);
    K1 = K(:,1:4);
    K2 = K(:,5:6);
else
    Q = [1 0 0 0 0 0;
        0 0.00 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 0.00 0 0;
        0 0 0 0 10000 0;
        0 0 0 0 0 10000];
    R = 0.2*eye(2);
    [K,P_lqr,eig_lqr] = lqr(Aaug,Baug,Q,R);
    K1 = K(:,1:4);
    K2 = K(:,5:6);
end

deltaT = 0.001;
deltaY = y_estimated - y_equ;
deltaX = qout(end,:)' - x_equ;
e = e_prev + deltaT*(deltaY - delta_y_des);

% Estimator
d_deltaXe = (A-F*C)*deltaXe_prev + F*(y_measured - y_equ) + B*(U-T_equ);
deltaXe_prev = deltaXe_prev + d_deltaXe*deltaT; 


% Plant Input
U = -K1*deltaXe_prev - K2*e + T_equ;


%U = -K1*deltaX -K2*e + T_equ; 
e_prev = e;


y_estimated = C*deltaXe_prev + y_equ;
x_endpoint_estimated = l1*cos(y_estimated(1)) + l2*cos(y_estimated(1) + y_estimated(2));
y_endpoint_estimated = l1*sin(y_estimated(1)) + l2*sin(y_estimated(1) + y_estimated(2));
xy_endpoint_estimated = [x_endpoint_estimated; y_endpoint_estimated];
