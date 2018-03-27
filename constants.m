%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c1=0.001;%8;%link 1 friction coeffecient
c2=0.001;%6;%link 2 friction coeffecient
l1=0.2121; %link 1 length
l2=0.0708; %link 2 length
m1=1;%2.5*l1;%link 1 mass
m2=1;%2.5*l2;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars
pA = [0.1 0.2];
pB = [0.2 0.2]; 
pC = [0.2 0.1]; pD = [0.1 0.1];

D = (pA(1)^2 + pA(2)^2 - l1^2 - l2^2)/(2*l1*l2);
Ath2 = atan2(sqrt(1-D^2), D);
Ath1 = atan2(pA(2),pA(1)) - (acos((pA(1)^2 + pA(2)^2 + l1^2 - l2^2)/(2*l1*sqrt(pA(1)^2 + pA(2)^2))));

D = (pB(1)^2 + pB(2)^2 - l1^2 - l2^2)/(2*l1*l2);
Bth2 = atan2(-sqrt(1-D^2), D);
Bth1 = atan2(pB(2),pB(1)) - (-acos((pB(1)^2 + pB(2)^2 + l1^2 - l2^2)/(2*l1*sqrt(pB(1)^2 + pB(2)^2))));

D = (pC(1)^2 + pC(2)^2 - l1^2 - l2^2)/(2*l1*l2);
Cth2 = atan2(-sqrt(1-D^2), D);
Cth1 = atan2(pC(2),pC(1)) - (-acos((pC(1)^2 + pC(2)^2 + l1^2 - l2^2)/(2*l1*sqrt(pC(1)^2 + pC(2)^2))));

D = (pD(1)^2 + pD(2)^2 - l1^2 - l2^2)/(2*l1*l2);
Dth2 = atan2(-sqrt(1-D^2), D);
Dth1 = atan2(pD(2),pD(1)) - (-acos((pD(1)^2 + pD(2)^2 + l1^2 - l2^2)/(2*l1*sqrt(pD(1)^2 + pD(2)^2))));

x_0=[Ath1,0,Ath2,0]';%x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
tau_0=[0.1,0.1]'; %initial torque
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Declare all your variables here, prefix with my_ %Feel Free to add to or remove these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time=0;
my_angle_vector=[0 0]';
my_state_estimate_vector=[0 0 0 0]';
my_some_variable_a=0;
my_some_variable_b=0;
my_goalpoints = [pA; pB; pC; pD]; % [A B C D]
d_wpts = 0.02;
w_pt = 2;
my_waypoints = [linspace( Ath1, Bth1, 10 );
                linspace( Ath2, Bth2, 10 )];%[min(Ath1,Bth1):0.05:max(Ath1,Bth1)]
my_bounds = [0 0; 0 0.22; 0.22 0.22; 0.22 0; 0 0];
x_lin = x_0;
deltaXe_prev = x_0 - x_lin;
e_prev = [x_0(1); x_0(3)] - my_waypoints(:,1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

