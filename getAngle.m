function [th1 th2] = getAngle(x, y, l1, l2)
    D = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
    th2 = atan2(sqrt(1-D^2), D);
    th1 = atan2(y,x) - (acos((x^2 + y^2 + l1^2 - l2^2)/(2*l1*sqrt(x^2 + y^2))));
end