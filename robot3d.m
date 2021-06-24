function robot3d(robotInWorld, robotMotors)
%-------------------------------------------------------------------------%
%                                Commande                                 %
%-------------------------------------------------------------------------%
psi = robotInWorld.psi;
theta = robotInWorld.theta;
robot_pos = [robotInWorld.x; robotInWorld.y; robotInWorld.z; psi; theta];
Df = -robotMotors.delta1;% front paddles
Dr = robotMotors.delta2;% rear palles
Dru = robotMotors.deltaR;% rudder 
%-------------------------------------------------------------------------%
%                             Robot Config                                %
%-------------------------------------------------------------------------%
global robotDim

robot_padlles = ...
[robotDim.length/4, -robotDim.length/4, robotDim.length/4, -robotDim.length/4, -robotDim.length/2;
 robotDim.width/2, robotDim.width/2, -robotDim.width/2, -robotDim.width/2, 0;
 0,               0,               0,               0,               0];


%-------------------------------------------------------------------------%
%                                3D-Plot                                  %
%-------------------------------------------------------------------------%

hold on
axis equal;
% axis([-.5 .5 -.5 .5 -.5 .5]);
grid on;
xlabel('X-axis / m');
ylabel('Y-axis / m');
zlabel('Z-axis / m');
view([2,-2,2]);
set(gca,'Ydir','reverse','Zdir','reverse')

N = [cos(theta)*cos(psi); sin(psi); -cos(psi)*sin(theta)];
O = [-cos(theta)*sin(psi); cos(psi); sin(theta)*sin(psi)];
A = [sin(theta); 0; cos(theta)];
pos = [robot_pos(1); robot_pos(2); robot_pos(3)];
T = [N,O,A,pos];
T = [T; [0 0 0 1]]; % homogenisation


Pa = [-robotDim.length/2; robotDim.width/2; robotDim.height/2; 1]; aw = T*Pa;
Pb = [robotDim.length/2; robotDim.width/2; robotDim.height/2; 1]; bw = T*Pb;
Pc = [robotDim.length/2; -robotDim.width/2; robotDim.height/2; 1]; cw = T*Pc;
Pd = [-robotDim.length/2; -robotDim.width/2; robotDim.height/2; 1]; dw = T*Pd;

PA = [-robotDim.length/2; robotDim.width/2; -robotDim.height/2; 1]; Aw = T*PA;
PB = [robotDim.length/2; robotDim.width/2; -robotDim.height/2; 1]; Bw = T*PB;
PC = [robotDim.length/2; -robotDim.width/2; -robotDim.height/2; 1]; Cw = T*PC;
PD = [-robotDim.length/2; -robotDim.width/2; -robotDim.height/2; 1]; Dw = T*PD;

x = [0.3; 0; 0; 1]; X = T*x;
y = [0; 0.3; 0; 1]; Y = T*y;
z = [0; 0; 0.3; 1]; Z = T*z;

% body frame
plot3([robot_pos(1),X(1)],[robot_pos(2),X(2)],[robot_pos(3),X(3)],'-r','LineWidth',.5); % axis X 
plot3(X(1),X(2),X(3),'>r');
plot3([robot_pos(1),Y(1)],[robot_pos(2),Y(2)],[robot_pos(3),Y(3)],'-g','LineWidth',.5); % axis Y
plot3(Y(1),Y(2),Y(3),'>g');
plot3([robot_pos(1),Z(1)],[robot_pos(2),Z(2)],[robot_pos(3),Z(3)],'-c','LineWidth',.5); % axis Z
plot3(Z(1),Z(2),Z(3),'>c');

% upper & lower surface
plot3([aw(1),bw(1),cw(1),dw(1),aw(1),Aw(1),Bw(1),Cw(1),Dw(1),Aw(1)],...
      [aw(2),bw(2),cw(2),dw(2),aw(2),Aw(2),Bw(2),Cw(2),Dw(2),Aw(2)],...
      [aw(3),bw(3),cw(3),dw(3),aw(3),Aw(3),Bw(3),Cw(3),Dw(3),Aw(3)],...
      '-k', 'LineWidth', .5);
plot3([bw(1),Bw(1)],...
      [bw(2),Bw(2)],...
      [bw(3),Bw(3)],'-k','LineWidth',.5);
plot3([cw(1),Cw(1)],...
      [cw(2),Cw(2)],...
      [cw(3),Cw(3)],'-k','LineWidth',.5); 
plot3([dw(1),Dw(1)],...
      [dw(2),Dw(2)],...
      [dw(3),Dw(3)],'-k','LineWidth',.5);

% 4 paddles & rudder
P1 = [robot_padlles(1,1); robot_padlles(2,1); robot_padlles(3,1); 1]; p1 = T*P1;
P2 = [robot_padlles(1,2); robot_padlles(2,2); robot_padlles(3,2); 1]; p2 = T*P2;
P3 = [robot_padlles(1,3); robot_padlles(2,3); robot_padlles(3,3); 1]; p3 = T*P3;
P4 = [robot_padlles(1,4); robot_padlles(2,4); robot_padlles(3,4); 1]; p4 = T*P4;
P5 = [robot_padlles(1,5); robot_padlles(2,5); robot_padlles(3,5); 1]; p5 = T*P5;

plot3(p1(1), p1(2), p1(3), 'o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');
plot3(p2(1), p2(2), p2(3), 'o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');
plot3(p3(1), p3(2), p3(3), 'o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');
plot3(p4(1), p4(2), p4(3), 'o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');
plot3(p5(1), p5(2), p5(3), 'o','Color','b','MarkerSize',3,'MarkerFaceColor','#D9FFFF');

P11 = [P1(1) + cos(Df)*robotDim.paddlelength/2; P1(2); P1(3) - sin(Df)*robotDim.paddlelength/2; 1]; p11 = T*P11;
P12 = [P1(1) - cos(Df)*robotDim.paddlelength/2; P1(2); P1(3) + sin(Df)*robotDim.paddlelength/2; 1]; p12 = T*P12;
P13 = [P1(1); P1(2)+robotDim.paddlelength/2; P1(3); 1]; p13 = T*P13;

P21 = [P2(1) + cos(Dr)*robotDim.paddlelength/2; P2(2); P2(3) - sin(Dr)*robotDim.paddlelength/2; 1]; p21 = T*P21;
P22 = [P2(1) - cos(Dr)*robotDim.paddlelength/2; P2(2); P2(3) + sin(Dr)*robotDim.paddlelength/2; 1]; p22 = T*P22;
P23 = [P2(1); P2(2)+robotDim.paddlelength/2; P2(3); 1]; p23 = T*P23;

P31 = [P3(1) + cos(Df)*robotDim.paddlelength/2; P3(2); P3(3) - sin(Df)*robotDim.paddlelength/2; 1]; p31 = T*P31;
P32 = [P3(1) - cos(Df)*robotDim.paddlelength/2; P3(2); P3(3) + sin(Df)*robotDim.paddlelength/2; 1]; p32 = T*P32;
P33 = [P3(1); P3(2)-robotDim.paddlelength/2; P3(3); 1]; p33 = T*P33;

P41 = [P4(1) + cos(Dr)*robotDim.paddlelength/2; P4(2); P4(3) - sin(Dr)*robotDim.paddlelength/2; 1]; p41 = T*P41;
P42 = [P4(1) - cos(Dr)*robotDim.paddlelength/2; P4(2); P4(3) + sin(Dr)*robotDim.paddlelength/2; 1]; p42 = T*P42;
P43 = [P4(1); P4(2)-robotDim.paddlelength/2; P4(3); 1]; p43 = T*P43;

% D1 = [P5(1); P5(2); P5(3)-robotDim.height/2; 1]; d1 = T*D1;
% D2 = [P5(1); P5(2); P5(3)+robotDim.height/2; 1]; d2 = T*D2;
% D3 = [P5(1)-cos(Dru)*robotDim.paddlelength/2; P5(2)+sin(Dru)*robotDim.paddlelength/2; P5(3)+robotDim.height/2; 1]; d3 = T*D3;
% D4 = [P5(1)-cos(Dru)*robotDim.paddlelength/2; P5(2)+sin(Dru)*robotDim.paddlelength/2; P5(3)-robotDim.height/2; 1]; d4 = T*D4;
D1 = [-robotDim.length/2; 0; -robotDim.height/2; 1]; d1 = T*D1;
D2 = [-robotDim.length/2; 0; robotDim.height/2; 1]; d2 = T*D2;
D3 = [-cos(Dru)*robotDim.paddlelength/2-robotDim.length/2; sin(Dru)*robotDim.paddlelength/2; robotDim.height/2; 1]; d3 = T*D3;
D4 = [-cos(Dru)*robotDim.paddlelength/2-robotDim.length/2; sin(Dru)*robotDim.paddlelength/2; -robotDim.height/2; 1]; d4 = T*D4;



vert = [p11(1), p11(2), p11(3);
        p12(1), p12(2), p12(3);
        p13(1), p13(2), p13(3);
        
        p21(1), p21(2), p21(3);
        p22(1), p22(2), p22(3);
        p23(1), p23(2), p23(3);
        
        p31(1), p31(2), p31(3);
        p32(1), p32(2), p32(3);
        p33(1), p33(2), p33(3);
        
        p41(1), p41(2), p41(3);
        p42(1), p42(2), p42(3);
        p43(1), p43(2), p43(3)];
        
     vertR = [d1(1), d1(2), d1(3);
        d2(1), d2(2), d2(3);
        d3(1), d3(2), d3(3);
        d4(1), d4(2), d4(3)];
        
       
fac = [1 2 3; 4 5 6; 7 8 9; 10 11 12];
patch('Vertices',vert,'Faces',fac,'FaceColor','green');
patch('Vertices',vertR,'Faces',[1 2 3 4],'FaceColor','c');

str = ['PosX =', num2str(robot_pos(1))];
text(robot_pos(1)+0.2, robot_pos(2), robot_pos(3)+0.2, [str, 'm']);
str = ['PosY =', num2str(robot_pos(2))];
text(robot_pos(1)+0.2, robot_pos(2), robot_pos(3)+0.15, [str, 'm']);
str = ['PosZ =', num2str(robot_pos(3))];
text(robot_pos(1)+0.2, robot_pos(2), robot_pos(3)+0.1, [str, 'm']);
drawnow


  