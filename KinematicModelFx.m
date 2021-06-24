function robotInWorld = KinematicModelFx(robotState, robotInWorld)
global const
tspan = [0 const.dt];

x = robotInWorld.x;
y = robotInWorld.y;
z = robotInWorld.z;
psi = robotInWorld.psi;
theta = robotInWorld.theta;
u = robotState.u;
v = robotState.v;
r = robotState.r;
w = robotState.w;
q = robotState.q;
y0 = [x y psi];
yy0 = [z theta];
opts = odeset('RelTol',1e-5,'AbsTol',1e-7);
uh = u * cos(theta);
wh = w * sin(theta);
uv = u * cos(psi);
vv = v * sin(psi);
[psi, xypsi] = ode45(@(psi, xypsi) H(xypsi, uh, v, wh, r), tspan, y0);
xypsi = xypsi(end,:);
[theta, ztheta] = ode45(@(theta, ztheta) V(ztheta, uv, vv, w, q), tspan, yy0);
ztheta =ztheta(end,:);
robotInWorld.x = xypsi(1);
robotInWorld.y = xypsi(2);
robotInWorld.z = ztheta(1);
robotInWorld.psi = xypsi(3);
robotInWorld.theta = ztheta(2);
    function xypsi_dot = H(xypsi, uh, v, wh, r)
        xypsi_dot = zeros(3,1);
        xypsi_dot(1) = u*cos(xypsi(3)) - v*sin(xypsi(3));
        xypsi_dot(2) = u*sin(xypsi(3)) + v*cos(xypsi(3));
        xypsi_dot(3) = r;
    end 
    function ztheta_dot = V(ztheta, uv,vv, w, q)
       ztheta_dot = zeros(2,1);
       ztheta_dot(1) = u*sin(-ztheta(2)) + w*cos(ztheta(2));
       ztheta_dot(2) = q;
    end 
end