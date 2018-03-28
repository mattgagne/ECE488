%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c1=8;%8;%link 1 friction coeffecient
c2=8;%6;%link 2 friction coeffecient
l1=0.2121; %link 1 length
l2=0.0879; %link 2 length
m1=2.5*l1;%link 1 mass
m2=2.5*l2;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars
measurement_std_dev = deg2rad(0.333);
pA = [0.1 0.2];
pB = [0.2 0.2]; 
pC = [0.2 0.1]; pD = [0.1 0.1];
num_waypts = 5;
my_waypts_xy = [linspace( pA(1), pB(1), num_waypts ) linspace( pB(1), pC(1), num_waypts ) linspace( pC(1), pD(1), num_waypts ) linspace( pD(1), pA(1), num_waypts );
                linspace( pA(2), pB(2), num_waypts ) linspace( pB(2), pC(2), num_waypts ) linspace( pC(2), pD(2), num_waypts ) linspace( pD(2), pA(2), num_waypts )];
for i = 1:size(my_waypts_xy,2)
    [my_waypts_ang(1,i) my_waypts_ang(2,i)] = getAngle(my_waypts_xy(1,i), my_waypts_xy(2,i), l1, l2);
end
x_0=[my_waypts_ang(1,1),0,my_waypts_ang(2,1),0]';%x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
tau_0=[0.1,0.1]'; %initial torque
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Declare all your variables here, prefix with my_ %Feel Free to add to or remove these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time=0;
my_angle_vector=[0 0]';
my_state_estimate_vector=[0 0 0 0]';
my_some_variable_a=0;
my_some_variable_b=0;
my_goalpoints = [pA; pB; pC; pD]; % [A B C D]
waypt = 2;
my_bounds = [0 0; 0 0.22; 0.22 0.22; 0.22 0; 0 0];
x_lin = x_0;
deltaXe_prev = x_0 - x_lin;
e_prev = [x_0(1); x_0(3)] - my_waypts_ang(:,1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

default_time_steps_to_wait = 0.5/0.001;
current_time_steps_waited = default_time_steps_to_wait;
useSettlingController = false;