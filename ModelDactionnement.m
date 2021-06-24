% in this simulation we considered all the drag coefficients are constant
% which means they won't change with different AOA (angle of attack): alpha
% and according to the fluid dynamique we difined the drag coefficient
% of our paddles (streamlined body) cd_p equals to 0 
% as for the robot (half-sphere body) cd_robot = 0.42
% note the drag equation : Fd = 1/2 * rho * v * v * cd * A
% rudder == symetric airfoil
% for simplification :
% lift coefficient is defined proportionally to alpha

function Forces = ModelDactionnement(robotMotors, u)
global robotDim const
rho = const.rho;

% cr = abs(delta);
% Ap = robotDim.paddleSurf * sin(alpha);

    function F = getLift(delta)
        cr = 0.5*abs(delta);
        Ap = robotDim.paddleSurf * sin(delta);
        F = 0.5*rho*u*abs(u)*cr*Ap;
    end 

Forces.F1 = getLift(robotMotors.delta1);
Forces.F2 = getLift(-robotMotors.delta2);
Forces.F3 = Forces.F1;
Forces.F4 = Forces.F2;
Forces.Fr = getLift(robotMotors.deltaR);
end