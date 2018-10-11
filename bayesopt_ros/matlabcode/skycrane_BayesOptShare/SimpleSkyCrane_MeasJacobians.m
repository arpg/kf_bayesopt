%SimpleSkyCrane_MeasJacobians.m

function [YXJacOut,YUJacOut] = SimpleSkyCrane_MeasJacobians(xIn,uIn,vehparams)

nStates = 6;
nInputs = 2;
nMeas = 4; %[xi,z,thetadot,xidoubledot]

YXJacOut = zeros(nMeas,nStates);
YUJacOut = zeros(nMeas,nInputs); 

%%vehicle states to linearize about
% xi = xIn(1);
xidot = xIn(2);
% z = xIn(3);
zdot = xIn(4);
theta = xIn(5);
% thetadot = xIn(6);

%%thrust inputs to linearize about
T1 = uIn(1);
T2 = uIn(2);

%%trig functions:
cosbeta = cos(vehparams.beta);
sinbeta = sin(vehparams.beta);
costheta = cos(theta);
sintheta = sin(theta);
%%dimensional constants
cd = 0.5*vehparams.rho*vehparams.C_D;
nc_m = -0.5*vehparams.C_D*vehparams.rho/(vehparams.m_f+vehparams.m_b);
oo_mt = 1/(vehparams.m_f+vehparams.m_b);
%%compute drag force
alpha = atan2(zdot,xidot); 
Aexposed = vehparams.Aside*cos(theta - alpha) + vehparams.Abot*sin(theta-alpha);
Vt = sqrt(xidot^2 + zdot^2);
Fdxi = 0.5 * vehparams.rho * vehparams.C_D *Aexposed * xidot*Vt; 
%%area trig terms:
Aside_cthetaaalpha = vehparams.Aside*cos(theta-alpha);
Aside_sthetaalpha = vehparams.Aside*sin(theta-alpha);
Abot_cthetaalpha  = vehparams.Abot*cos(theta-alpha);
Abot_sthetaalpha = vehparams.Abot*sin(theta-alpha);

%%-----------Compute easy Meas Jacobians w.r.t. state
YXJacOut(1,1) = 1;
YXJacOut(2,3) = 1;
YXJacOut(3,6) = 1;
%%----Compute Meas Jacobian of xi double dot w.r.t. all states (via NL Dynamics):
if Vt > 0  %%look out for corner case; o'wise derivs are zero
%%d(dotxi)/d(xidot):
YXJacOut(4,2) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((2*xidot^2 + zdot^2)/Vt) ...
    + (xidot*zdot/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));    
    
%%d(dotxi)/d(zdot):
YXJacOut(4,4) = ...
    nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)...
    *((xidot*zdot)/Vt) ...
    + (xidot^2/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
end

%%d(dotxi)/d(theta):
YXJacOut(4,5) = ...
    oo_mt*(T1*(cosbeta*costheta - sinbeta*sintheta) ...
         + T2*(cosbeta*costheta + sinbeta*sintheta)  ...
         + cd*xidot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha));

%%----Compute Meas Jacobian of xi double dot  w.r.t. input thrusts (via NL Dynamics):     
YUJacOut(4,1) = oo_mt*(cosbeta*sintheta + sinbeta*costheta);
YUJacOut(4,2) = oo_mt*(cosbeta*sintheta - sinbeta*costheta); 
end