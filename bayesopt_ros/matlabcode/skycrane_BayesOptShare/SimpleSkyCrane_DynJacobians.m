%SimpleSkyCrane_DynJacobians.m
%%%Compute Jacobians for Simple Sky Crane system.
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
function [XJacout,UJacout,WJacout] = SimpleSkyCrane_DynJacobians(xIn,uIn,wIn,vehparams)

nStates = 6;
nInput = 2;
nDists = 3;

XJacout = zeros(nStates,nStates);
UJacout = zeros(nStates,nInput);
WJacout = zeros(nStates,nDists);

%%states to linearize about:
%xi = xIn(1);
xidot = xIn(2);
%z = x(3);
zdot = xIn(4);
theta = xIn(5);
%thetadot = xIn(6);
%%control inputs to linearize about
T1 = uIn(1);
T2 = uIn(2);

%%Compute common constants and variables:
%%angle of attack: 
alpha = atan2(zdot,xidot); %%check!! unwrapping issues???
%%total velocity:
Vt = sqrt(xidot^2 + zdot^2);
%%trig functions:
cosbeta = cos(vehparams.beta);
sinbeta = sin(vehparams.beta);
costheta = cos(theta);
sintheta = sin(theta);

%%dimensional constants
cd = 0.5*vehparams.rho*vehparams.C_D;
nc_m = -0.5*vehparams.C_D*vehparams.rho/(vehparams.m_f+vehparams.m_b);
oo_mt = 1/(vehparams.m_f+vehparams.m_b);
%%area trig terms:
Aside_cthetaaalpha = vehparams.Aside*cos(theta-alpha);
Aside_sthetaalpha = vehparams.Aside*sin(theta-alpha);
Abot_cthetaalpha  = vehparams.Abot*cos(theta-alpha);
Abot_sthetaalpha = vehparams.Abot*sin(theta-alpha);

%%%------------------------------------------------------------------%%
%%%------------------------------------------------------------------%%

%%State Jacobian (non-zero terms only):

XJacout(1,2) = 1; %%d(xidot)/d(xidot)
XJacout(3,4) = 1; %%d(zdot)/d(zdot)
XJacout(5,6) = 1; %%d(thetadot)/d(thetadot)

if Vt > 0  %%look out for corner case; o'wise derivs are zero
%%d(dotxi)/d(xidot):
XJacout(2,2) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((2*xidot^2 + zdot^2)/Vt) ...
    + (xidot*zdot/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));    
    
%%d(dotxi)/d(zdot):
XJacout(2,4) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((xidot*zdot)/Vt) ...
    + (xidot^2/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
end

%%d(dotxi)/d(theta):
XJacout(2,5) = ...
    oo_mt*(T1*(cosbeta*costheta - sinbeta*sintheta) ...
         + T2*(cosbeta*costheta + sinbeta*sintheta)  ...
         + cd*xidot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha));
%%%------------------------------------------------------------------%%

if Vt > 0  %%look out for corner case; o'wise derivs are zero
%%d(dotz)/d(xidot):
XJacout(4,2) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((xidot*zdot)/Vt) ...
    + (zdot^2/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));

%%d(dotz)/d(zdot):
XJacout(4,4) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((2*xidot^2 + zdot^2)/Vt) ...
    + (zdot*xidot/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
end

%%d(dotz)/d(theta):
XJacout(4,5) = ...
    oo_mt*(-T1*(cosbeta*sintheta + sinbeta*costheta) ...
          + T2*(sinbeta*costheta - cosbeta*sintheta)  ...
          + cd*zdot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha));
%%%------------------------------------------------------------------%%

%%Control input Jacobians (non-zero terms only): 
%%d(ddotxi)/d(Ti): i=1,2
UJacout(2,1) = oo_mt*(cosbeta*sintheta + sinbeta*costheta);
UJacout(2,2) = oo_mt*(cosbeta*sintheta - sinbeta*costheta);

%%d(ddotz)/d(Ti): i=1,2
UJacout(4,1) = oo_mt*(cosbeta*costheta - sinbeta*sintheta);
UJacout(4,2) = oo_mt*(cosbeta*costheta + sinbeta*sintheta);

%%d(ddottheta)/d(Ti): i=1,2
UJacout(6,1) = (1/vehparams.Ieta)*(cosbeta*vehparams.w_b*0.5 - sinbeta*vehparams.hcm);
UJacout(6,2) = -UJacout(6,1);

%%%------------------------------------------------------------------%%

%%Disturbance input Jacobians (non-zero terms only):
WJacout(2,1) = 1;
WJacout(4,2) = 1;
WJacout(6,3) = 1;

end