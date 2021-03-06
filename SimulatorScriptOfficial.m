%this is the general format of the simulator script
clc;
format long;
close all;
clear all;
constants;
%constants2;


%initializing code

%initial conditions   
X0=x_0;
U=tau_0;

u_integral = U'*U;

[tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1,m2,g,c1,c2),[0 0.001],X0);
q=qout(end,[1,3])';
    
% Additions for testing
%y_des = [Bth1 Bth2]';
ts = 0:0.001:8;
%e_prev = q - y_des;
% End of additions


for t=0.001:0.001:10
   t;
   
   if (waypt > size(my_waypts_ang, 2))
       break;
   end
   %check if robot meets requirements

   RobotControllerScript %your script is used here.
   u_integral = u_integral + U'*U;

   
   [tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1,m2,g,c1,c2),[t t+0.001],qout(end,:));
   q= [q qout(end,[1,3])'];
   
 end
 
 %calculate energy/time, etc...

%  %% Additions for visualization
%  figure(1);
%  subplot(2,1,1);
%  plot(ts, q(1,:));
%  subplot(2,1,2);
%  plot(ts, q(2,:));
 
 
%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

for i = 1:125:size(q,2)
    figure(2);clf; hold on;
    x1 = l1*cos(q(1,i));
    y1 = l1*sin(q(1,i));
    x2 = l1*cos(q(1,i)) + l2*cos(q(1,i) + q(2,i));
    y2 = l1*sin(q(1,i)) + l2*sin(q(1,i) + q(2,i));
    plot([0,x1],[0,y1],'r','LineWidth',2);
    plot([x1,x2],[y1,y2],'b','LineWidth',2);
    % Plot target
    %plot([0 l1*cos(y_des(1))], [0 l1*sin(y_des(1))],'black');
    %plot([l1*cos(y_des(0)) l1*cos(y_des(1)) + l2*cos(y_des(1) + y_des(2))], [l1*sin(y_des(1)) l1*sin(y_des(1)) + l2*sin(y_des(1) + y_des(2))],'black');
    scatter(my_goalpoints(:,1),my_goalpoints(:,2),15,'filled');
    plot(my_bounds(:,1),my_bounds(:,2),'--r','LineWidth',2);
    xlim(1*[0,0.40]);
    ylim(1*[-.17,0.23]);
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
end

close(vidObj);

 % End of additions
 
t
u_integral * 0.001
