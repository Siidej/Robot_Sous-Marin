function robotState = DynamicModel(Forces, robotState)

global robotDim F const
m = robotDim.weight;
tspan = [0 const.dt];
% for the robot (half-sphere body) cd_robot = 0.42
cd = 0.42;
cd_c = 1.05; % drag coeff of a cube 
rho = const.rho;
Au = robotDim.width * robotDim.height;
Aw = robotDim.width * robotDim.length;
Av = robotDim.height * robotDim.length;
Fr = Forces.Fr;
F1 = Forces.F1;
F2 = Forces.F2;
F3 = Forces.F3;
F4 = Forces.F4;

Lr = robotDim.length/2;
Ld = sqrt((robotDim.length/6)^2+(robotDim.width/2)^2);
%-------------------------------------------------------------------------%
%                                Horizontal                               %
%-------------------------------------------------------------------------%
% m*u_dot = F - Du;
% Du = 1/2 * rho * u * u * cd * Au; 
    [~, u] = ode45(@(t, u) (F-0.5*rho*u*abs(u)*cd*Au)/m, tspan, robotState.u);
    robotState.u = u(end);

% m*v_dot = Fr - Dv;
% Dv = 1/2 * rho * v * v * cd_c * Av; 

    [~, v] = ode45(@(t, v) (Fr-0.5*rho*v*abs(v)*cd_c*Av)/m, tspan, robotState.v);
    robotState.v = v(end);


% m*r_dot = Fr*d - Dr;
% Dr = 1/2 * rho * r * r * cd_c * Av/2; 

    [~, r] = ode45(@(t, r) (Fr*Lr-0.5*rho*r*abs(r)*cd_c*Av)/m, tspan, robotState.r);
    robotState.r = r(end);


%-------------------------------------------------------------------------%
%                                  Vertical                               %
%-------------------------------------------------------------------------%
% m*w_dot = F1+F2+F3+F4-Dw + G; %
% Dw = 1/2 * rho * w * w * cd_c * Aw; ;
SF = F1+F2+F3+F4;

    [~, w] = ode45(@(t, w) (SF-0.5*rho*w*abs(w)*cd_c*Aw)/m, tspan, robotState.w);
    robotState.w = w(end);

% m*q_dot = Ld*F1+Ld*F2+Ld*F3+Ld*F4-Dr + G; %
% Dr = 1/2 * rho * w * w * cd_c * Aw/2; ;
Tor = Ld*(F2+F4-F1-F3);

    [~, q] = ode45(@(t, q) (Tor-0.5*rho*q*abs(q)*cd_c*Aw)/m, tspan, robotState.q);
    robotState.q = q(end);


end