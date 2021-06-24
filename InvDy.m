function ForcesC = InvDy(robotStateC)

global robotDim F const
m = robotDim.weight;
rho = const.rho;
cd = 0.42;
% cd_c = 1.05; % drag coeff of a cube 
cd_c = 0.5; % drag coeff of a cube 

Au = robotDim.width * robotDim.height;
Aw = robotDim.width * robotDim.length;
Av = robotDim.height * robotDim.length;
 
% Lr = robotDim.length/2;
Ld = sqrt((robotDim.length/6)^2+(robotDim.width/2)^2);

u = robotStateC.u;
v = robotStateC.v;
r = robotStateC.r;
w = robotStateC.w;
q = robotStateC.q;

ForcesC.Fr = v*m + 0.5*rho*v*abs(v)*cd_c*Av;
ForcesC.F1 = ((w*m + 0.5*rho*w*abs(w)*cd_c*Aw)/2  - (m*q + 0.5*rho*q*abs(q)*cd_c*Aw)/(2*Ld))/2 ;
ForcesC.F2 = (w*m + 0.5*rho*w*abs(w)*cd_c*Aw)/2 - ForcesC.F1;
ForcesC.F3 = ForcesC.F1;
ForcesC.F4 = ForcesC.F2;
ForcesC.F = u*m + 0.5*rho*u*abs(u)*cd*Au;