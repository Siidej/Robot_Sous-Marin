clear 
close all 
clc

global robotDim const F
robotDim.length = 0.4; %m
robotDim.width = 0.3; %m
robotDim.height = 0.1; %m
robotDim.paddlelength = robotDim.length/2.6667; %m
robotDim.paddleSurf = robotDim.paddlelength^2; %m^2
robotDim.weight = 1; %kg

F = 100;
const.rho = 1030;%kg/(m^3) mass density of seawater
const.dt = 0.05;

robotInWorld.x = 0;
robotInWorld.y = 0;
robotInWorld.z = 0;
robotInWorld.psi = 0; % yaw
robotInWorld.theta = 0;% pitch

robotState.u = 0;
robotState.v = 0;
robotState.w = 0;
robotState.r = 0;
robotState.q = 0;

%  Roll = 0 => delta1 = -delta3 , delta2 = -delta4
robotMotors.delta1 = 0;
robotMotors.delta2 = -pi/6;
% robotMotors.delta3 = 0;
% robotMotors.delta4 = 0;
robotMotors.deltaR = -pi/6;

obj = VideoWriter('animation');
obj.Quality = 100;
obj.FrameRate = 20;
open(obj);

for t = 0:const.dt:10
    cla
    Forces = ModelDactionnement(robotMotors, robotState.u);
    robotState = DynamicModel(Forces, robotState)
    robotInWorld = KinematicModel(robotState, robotInWorld);
    robot3d(robotInWorld, robotMotors);
    f= getframe(gcf);
    writeVideo(obj,f);
    axis equal;

end
close(obj);


% obj = VideoWriter('traj');
% obj.Quality = 100;
% obj.FrameRate = 20;
% open(obj);
% for t = 0:const.dt:10
%     figure(2)
%     hold on 
%     grid on 
%     xlabel('X-axis / m');
%     ylabel('Y-axis / m');
%     zlabel('Z-axis / m');
%     view([2,-2,2]);
%     axis equal;
%     set(gca,'Ydir','reverse','Zdir','reverse')
%     Forces = ModelDactionnement(robotMotors, robotState.u);
%     robotState = DynamicModel1(Forces, robotState);
%     robotInWorld = KinematicModelFx(robotState, robotInWorld);
%     plot3(robotInWorld.x,  robotInWorld.y,  robotInWorld.z, '.');
%     f= getframe(gcf);
%     writeVideo(obj,f);
% end
% close(obj);

