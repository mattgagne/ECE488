function [Xd] = simulatorofficial(time,x,U,L1,L2,m1,m2,g,c1,c2)

    x

    syms q1dd q2dd;
    T1 = U(1);
    T2 = U(2);
    q1 = x(1);
    q1d = x(2);
    q2 = x(3);
    q2d = x(4);

    q1dd = (3*(4*L2*T1 - 4*L2*T2 - 4*L2*c1*q1d + 4*L2*c2*q2d - 6*L1*T2*cos(q2) + 6*L1*c2*q2d*cos(q2) + 2*L1*L2^2*m2*q1d^2*sin(q2) + 2*L1*L2^2*m2*q2d^2*sin(q2) - 2*L1*L2*g*m1*cos(q1) - 4*L1*L2*g*m2*cos(q1) + 3*L1*L2*g*m2*cos(q1 + q2)*cos(q2) + 3*L1^2*L2*m2*q1d^2*cos(q2)*sin(q2) + 4*L1*L2^2*m2*q1d*q2d*sin(q2)))/(4*L1^2*L2*m1 + 12*L1^2*L2*m2 - 9*L1^2*L2*m2*cos(q2)^2);

    q2dd = -(3*(4*L2^2*T1*m2 - 12*L1^2*T2*m2 - 4*L1^2*T2*m1 - 4*L2^2*T2*m2 + 4*L1^2*c2*m1*q2d - 4*L2^2*c1*m2*q1d + 12*L1^2*c2*m2*q2d + 4*L2^2*c2*m2*q2d + 6*L1^2*L2*g*m2^2*cos(q1 + q2) - 4*L1*L2^2*g*m2^2*cos(q1) + 6*L1*L2*T1*m2*cos(q2) - 12*L1*L2*T2*m2*cos(q2) + 2*L1*L2^3*m2^2*q1d^2*sin(q2) + 6*L1^3*L2*m2^2*q1d^2*sin(q2) + 2*L1*L2^3*m2^2*q2d^2*sin(q2) + 3*L1*L2^2*g*m2^2*cos(q1 + q2)*cos(q2) + 6*L1^2*L2^2*m2^2*q1d^2*cos(q2)*sin(q2) + 3*L1^2*L2^2*m2^2*q2d^2*cos(q2)*sin(q2) - 6*L1*L2*c1*m2*q1d*cos(q2) + 12*L1*L2*c2*m2*q2d*cos(q2) - 6*L1^2*L2*g*m2^2*cos(q1)*cos(q2) + 2*L1^2*L2*g*m1*m2*cos(q1 + q2) - 2*L1*L2^2*g*m1*m2*cos(q1) + 2*L1^3*L2*m1*m2*q1d^2*sin(q2) + 4*L1*L2^3*m2^2*q1d*q2d*sin(q2) + 6*L1^2*L2^2*m2^2*q1d*q2d*cos(q2)*sin(q2) - 3*L1^2*L2*g*m1*m2*cos(q1)*cos(q2)))/(12*L1^2*L2^2*m2^2 + 4*L1^2*L2^2*m1*m2 - 9*L1^2*L2^2*m2^2*cos(q2)^2);
    
    Xd = [q1d; q1dd; q2d; q2dd];
end