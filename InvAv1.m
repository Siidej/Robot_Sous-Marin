function robotMotors = InvAv1(Forces, u, robotMotors)

global robotDim const
rho = const.rho;
Surf =robotDim.paddleSurf;

FM = [Forces.F1;Forces.F2;Forces.F3
     Forces.F4;Forces.Fr];
 

lb = [-pi/9, -pi/9, -pi/9]; 
ub = [pi/9, pi/9, pi/9];

% rng default % Reproducible initial point
opts = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
delta = fmincon(@(delta)0,[robotMotors.delta1, robotMotors.delta2, robotMotors.deltaR],[],[],[],[],lb,ub, @fminconstr, opts);

% if delta(1) > ub(1)
%     delta(1) = ub(1);
% end
% 
% if delta(2) > ub(2)
%     delta(2) = ub(2);
% end
% 
% if delta(3) > ub(3)
%     delta(3) = ub(3);
% end

robotMotors.delta1 = delta(1);
robotMotors.delta2 = delta(2);
robotMotors.delta3 = -delta(1);
robotMotors.delta4 = -delta(2);
robotMotors.deltaR = delta(3);

    function [c, ceq] = fminconstr(delta)
    %https://fr.mathworks.com/help/optim/ug/nonlinear-systems-with-constraints.html
    c = [];
    ceq = myFunc(delta);
    end 

    function D = myFunc(delta)
        D(1) = 0.5*rho*u*abs(u)*0.5*Surf*sin(delta(1)) - FM(1);
        D(2) = 0.5*rho*u*abs(u)*0.5*Surf*sin(delta(2)) - FM(2);
        D(3) = 0.5*rho*u*abs(u)*0.5*Surf*sin(delta(3)) - FM(5);
    end 

end 