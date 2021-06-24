function robotState = InvKin(pos_tar, robotInWorld)
gain = eye(5,5).*0.05;

% psi = robotInWorld.psi;
% theta = robotInWorld.theta;
% robot_pos = [robotInWorld.x; robotInWorld.y; robotInWorld.z; psi; theta];
% 
% N = [cos(theta)*cos(psi); sin(psi); -cos(psi)*sin(theta)];
% O = [-cos(theta)*sin(psi); cos(psi); sin(theta)*sin(psi)];
% A = [sin(theta); 0; cos(theta)];
% pos = [robot_pos(1); robot_pos(2); robot_pos(3)];
% T = [N,O,A,pos];
% T = [T; [0 0 0 1]]; % homogenisation
% T = pinv(T);

deltaV = [pos_tar.x-robotInWorld.x; pos_tar.y-robotInWorld.y; pos_tar.z-robotInWorld.z;...
          pos_tar.psi-robotInWorld.psi; pos_tar.theta-robotInWorld.theta];
% deltaV = T*deltaV;
deltaV = gain*deltaV;
% deltaV = T * [pos_tar.x;pos_tar.y;pos_tar.z;1];

M = [cos(deltaV(5))*cos(deltaV(4)) -sin(deltaV(4)) -sin(deltaV(5))*cos(deltaV(4)) 0 0;...
     cos(deltaV(5))*sin(deltaV(4)) cos(deltaV(4)) -sin(deltaV(5))*sin(deltaV(4)) 0 0;...
     cos(deltaV(4))*sin(-deltaV(5)) sin(-deltaV(5))*sin(deltaV(4)) cos(deltaV(5)) 0 0;...
     0 0 0 1 0;
     0 0 0 0 1];
 
State = pinv(M)*deltaV;

robotState.u = State(1);
robotState.v = State(2);
robotState.w = State(3);
robotState.r = State(4);
robotState.q = State(5);






