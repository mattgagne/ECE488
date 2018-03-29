%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
density = 1.4;
length_total = 0.75/density;


c1=6;%8;%link 1 friction coeffecient
c2=6;%6;%link 2 friction coeffecient
l1=3/5*length_total; %link 1 length
l2=2/5*length_total; %link 2 length
m1=density*l1;%link 1 mass
m2=density*l2;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars
measurement_std_dev = deg2rad(0.333);
pA = [0.1 0.2];
pB = [0.2 0.2]; 
pC = [0.2 0.1]; pD = [0.1 0.1];
num_waypts = 2;
my_waypts_xy = [logspace( log10(pA(1)), log10(pB(1)), num_waypts ) pB(1) logspace( log10(pB(1)), log10(pC(1)), num_waypts ) pC(1) logspace( log10(pC(1)), log10(pD(1)), num_waypts ) pD(1) logspace( log10(pD(1)), log10(pA(1)), num_waypts ) pA(1);
                logspace( log10(pA(2)), log10(pB(2)), num_waypts ) pB(2) logspace( log10(pB(2)), log10(pC(2)), num_waypts ) pC(2) logspace( log10(pC(2)), log10(pD(2)), num_waypts ) pD(2) logspace( log10(pD(2)), log10(pA(2)), num_waypts ) pA(2)];
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

waited_timesteps = 0;
xy_endpoint_estimated = [0;0];
y_estimated = [0;0];