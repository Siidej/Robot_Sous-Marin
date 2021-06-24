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

F = 0;
const.rho = 1020;%kg/(m^3) mass density of seawater
const.dt = 0.05;

robotInWorld.x = 0.1;
robotInWorld.y = 0.1;
robotInWorld.z = 0.1;
robotInWorld.psi = 0; % yaw
robotInWorld.theta = 0;% pitch

pos_tar.x = 1;
pos_tar.y = -1;
pos_tar.z = 1;
pos_tar.psi = 0;
pos_tar.theta = 0;

robotState.u = 0;
robotState.v = 0;
robotState.w = 0;
robotState.r = 0;
robotState.q = 0;

%  Roll = 0 => delta1 = -delta3 , delta2 = -delta4
robotMotors.delta1 = 0;
robotMotors.delta2 = 0;
% robotMotors.delta3 = 0;
% robotMotors.delta4 = 0;
robotMotors.deltaR = 0;

% 
% robotState = InvKin(pos_tar, robotInWorld);
% Forces = InvDy(robotState);
% robotMotors = InvAv(Forces, robotState.u)
obj = VideoWriter('animation');
obj.Quality = 100;
obj.FrameRate = 20;
open(obj);
% for t = 0:const.dt:20    
%     cla
%     robotStateC = InvKin(pos_tar, robotInWorld);
%     ForcesC = InvDy(robotStateC,robotState);
%     robotMotors = InvAv1(ForcesC, robotStateC.u, robotMotors);
%     F=ForcesC.F;
%     Forces = ModelDactionnement(robotMotors, robotStateC.u);
%     robotState = DynamicModel1(Forces, robotState);
%     robotInWorld = KinematicModel(robotState, robotInWorld)
% %     plot(t, robotInWorld.x, '*');
%     
%     robot3d(robotInWorld, robotMotors);
%      f= getframe(gcf);
%      writeVideo(obj,f);
%     
%     axis equal;
%     hold on
%     drawnow
% end
close(obj)

for t = 0:const.dt:20    
    robotStateC = InvKin(pos_tar, robotInWorld);
    ForcesC = InvDy(robotStateC)
    robotMotors = InvAv(ForcesC, robotStateC.u, robotMotors)
    F=ForcesC.F;
    Forces = ModelDactionnement(robotMotors, robotStateC.u);
    robotState = DynamicModel1(Forces, robotState);
    robotInWorld = KinematicModel(robotState, robotInWorld);
    plot3(robotInWorld.x,robotInWorld.y,robotInWorld.z, '.');  
    axis equal;
    hold on
    grid on 
    drawnow
end
