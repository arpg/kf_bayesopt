%SimpleSkyCrane_NLDynamics.m
%%%Nonlinear equations of motion (in standard form) 
%%%for simple Sky Crane system. 
%%%States and input definition:
%%%-x: [xi,xidot, z, zdot, theta, thetadot]^T
%%%-u: [T1,T2]^T
%%%-wtildeIn = [wtilde1, wtilde2, wtilde3]^T: disturbance forces and moments
%%%where xi = inertial horizontal position (m)
%%%         z = inertial vertical position (m)
%%%      theta= pitch angle (rads)
%%%         Ti = thruster i force (N)
%%%-vehparams: nominal vehicle parameters 
%%%         beta (thruster angle - rads, fixed)
%%%         g (3.711 - Martian gravitation acceleration in m/s^2, fixed) 
%%%         rho (0.02 - Martian atmosphere density, kg/m^3, fixed)
%%%         C_D (0.2 - drag coefficient, fixed)
%%%         m_b, m_f (vehicle body and fuel mass, kg, fixed)
%%%         w_b, h_b (vehicle width and height, m, fixed)
%%%         w_f, h_f (propellant housing width and height, m, fixed)
%%%         hcm      (vehicle center of mass height, m, fixed)
%%%         Ieta     (vehicle moment of inertia about eta axis, fixed)
%%%         Aside    (vehicle side area)
%%%         Abot     (vehicle bottom area)
function xdotOut = SimpleSkyCrane_NLDynamics(t,xIn,uIn,wtildeIn,vehparams)

xdotOut = zeros(6,1);

%%states:
%xi = xIn(1);
xidot = xIn(2);
%z = x(3);
zdot = xIn(4);
theta = xIn(5);
thetadot = xIn(6);
T1 = uIn(1);
T2 = uIn(2);

%%angle of attack: 
alpha = atan2(zdot,xidot); %%check!! unwrapping issues???
%%total velocity:
Vt = sqrt(xidot^2 + zdot^2);

%%drag forces:
Aexposed = vehparams.Aside*cos(theta - alpha) + vehparams.Abot*sin(theta-alpha);
Fdxi = 0.5 * vehparams.rho * vehparams.C_D *Aexposed * xidot*Vt; 
Fdz = 0.5 * vehparams.rho * vehparams.C_D *Aexposed * zdot*Vt;

%%compute xdot:
xdotOut(1) = xidot;
xdotOut(3) = zdot;
xdotOut(5) = thetadot;

xdotOut(2) = ...
 (T1*(cos(vehparams.beta)*sin(theta) + sin(vehparams.beta)*cos(theta)) ...
 +T2*(cos(vehparams.beta)*sin(theta) - sin(vehparams.beta)*cos(theta)) ...
 - Fdxi) /(vehparams.m_f + vehparams.m_b) + wtildeIn(1);

xdotOut(4) = ...
 (T1*(cos(vehparams.beta)*cos(theta) - sin(vehparams.beta)*sin(theta)) ...
 +T2*(cos(vehparams.beta)*cos(theta) + sin(vehparams.beta)*sin(theta)) ...
 - Fdz) /(vehparams.m_f + vehparams.m_b) -vehparams.g + wtildeIn(2);

xdotOut(6) = ...
    (1/vehparams.Ieta)*...
    ( (T1-T2)*cos(vehparams.beta)*vehparams.w_b*0.5 ...
    + (T2-T1)*sin(vehparams.beta)*(vehparams.hcm)) + wtildeIn(3);
end