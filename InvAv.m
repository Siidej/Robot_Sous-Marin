function robotMotors = InvAv(Forces, u, robotMotors)

global robotDim const
rho = const.rho;
Surf =robotDim.paddleSurf;
FM = [Forces.F1;Forces.F2;Forces.F3
     Forces.F4;Forces.Fr];
 
% options = optimoptions('fsolve'); 
% options.MaxIterations = 1000;
% options.MaxFunctionEvaluations = 5000;
lb = [-pi/9, -pi/9, -pi/9]; 
ub = [pi/9, pi/9, pi/9]; 

fun = @(delta) myFunc(delta, FM, Surf, u, rho);
delta  = lsqnonlin(fun, [robotMotors.delta1, robotMotors.delta2, robotMotors.deltaR],lb,ub);

    function D = myFunc(delta, FM, Surf, u, rho)
%         D(1) = 0.5*rho*u*abs(u)*abs(delta(1))*Surf*sin(delta(1)) - F(1);
%         D(2) = 0.5*rho*u*abs(u)*abs(delta(2))*Surf*sin(delta(2)) - F(2);
%         D(3) = 0.5*rho*u*abs(u)*abs(delta(3))*Surf*sin(delta(3)) - F(5);
        D(1) = 0.5*rho*u*u*0.5*Surf*sin(delta(1)) - FM(1);
        D(2) = 0.5*rho*u*u*0.5*Surf*sin(delta(2)) - FM(2);
        D(3) = 0.5*rho*u*u*0.5*Surf*sin(delta(3)) - FM(5);
    end 

robotMotors.delta1 = delta(1);
robotMotors.delta2 = delta(2);
robotMotors.delta3 = -delta(1);
robotMotors.delta4 = -delta(2);
robotMotors.deltaR = delta(3);
end 