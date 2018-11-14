%SimpleSkyCrane_NLMeasurements.m

function yOut = SimpleSkyCrane_NLMeasurements(xIn,uIn,vehparams)

% nStates = length(xIn);
% nInputs = length(uIn);
nMeas = 4;

yOut = zeros(nMeas,1);

%%vehicle states
xi = xIn(1);
xidot = xIn(2);
z = xIn(3);
zdot = xIn(4);
theta = xIn(5);
thetadot = xIn(6);
%%thrust inputs
T1 = uIn(1);
T2 = uIn(2);

%%compute drag force
alpha = atan2(zdot,xidot); 
Aexposed = vehparams.Aside*cos(theta - alpha) + vehparams.Abot*sin(theta-alpha);
Vt = sqrt(xidot^2 + zdot^2);
Fdxi = 0.5 * vehparams.rho * vehparams.C_D *Aexposed * xidot*Vt; 

%%assign easy measurements
yOut(1) = xi;
yOut(2) = z;
yOut(3) = thetadot;

%%compute xi doubledot using dynamics model:
yOut(4) = (T1*(cos(vehparams.beta)*sin(theta) + sin(vehparams.beta)*cos(theta)) ...
 +T2*(cos(vehparams.beta)*sin(theta) - sin(vehparams.beta)*cos(theta)) ...
 - Fdxi) /(vehparams.m_f + vehparams.m_b);
end