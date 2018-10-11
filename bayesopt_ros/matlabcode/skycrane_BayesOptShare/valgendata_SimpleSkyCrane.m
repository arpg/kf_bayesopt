%valgendata_SimpleSkyCrane.m

%%Generate, validate and store assignment data for 
%%Simple Sky Crane NL Dynamics with nominal specs. 

clc,clear,close all
rng(100)%% rng(304) %% 

%% 0. spec vehicle geometry and inertia properties
%%%-vehparams: vehicle parameters 
%%%         beta (thruster angle - rads, fixed)
%%%         g (3.711 - Martian gravitation acceleration in m/s^2, fixed) 
%%%         rho (0.02 - Martian atmosphere density, kg/m^3, fixed)
%%%         C_D (0.2 - drag coefficient, fixed)
%%%         m_b, m_f (vehicle body and fuel mass, kg, fixed)
%%%         w_b, h_b, d_b (vehicle width and height and depth, m, fixed)
%%%         w_f, h_f, d_f (propellant housing width and height and depth, m, fixed)
%%%         hcm      (vehicle center of mass height, m, fixed)
%%%         Ieta     (vehicle moment of inertia about eta axis, fixed)
%%%         Aside    (vehicle side area)
%%%         Abot     (vehicle bottom area)

vehparams.beta = 45 * pi/180;
vehparams.g = 3.711; 
vehparams.rho = 0.02;
vehparams.C_D = 0.2;
vehparams.m_b = 611+ 899; %%~mass of EDL system + rover (w/o heatshield or backshell)
vehparams.m_f = 390; 
vehparams.w_b = 3.2;
vehparams.h_b = 2.5;
vehparams.d_b = 2.9; %vehicle depth
vehparams.w_f = 1;
vehparams.h_f = 0.5;
vehparams.d_f = 1; %housing depth
vehparams.hcm = ... %%vehicle vertical center of mass (assuming body + housing mass at geo centroids)
    (vehparams.m_b*vehparams.h_b*0.5 - vehparams.m_f*vehparams.h_f*0.5)...
    /(vehparams.m_b+vehparams.m_f); 
vehparams.Ieta = (1/12)*...
    (vehparams.m_b*((vehparams.w_b)^2 + (vehparams.h_b)^2) ...
    +vehparams.m_f*((vehparams.w_f)^2 + (vehparams.h_f)^2) );
vehparams.Aside = vehparams.h_b*vehparams.d_b + vehparams.h_f*vehparams.d_f; 
vehparams.Abot = vehparams.w_b*vehparams.d_b + vehparams.w_f*vehparams.d_f; 


%% 1. simulate non-linear dynamics and data for nominal static hover
%%%Compute nominal thrusts for static hover equilibrium using 2 thrusters
%%(note: actual thrusters had thrust range of 400-3060 N for 4 thrusters)
Tnom = 0.5* vehparams.g * (vehparams.m_b + vehparams.m_f)/cos(vehparams.beta);

dt = 0.1;
tvec = 0:dt:50;
stateLabels = {'\xi (m)', '\xi dot (m/s)', 'z (m)','z dot (m/s)',...
               '\theta (rads)', '\theta dot (rad/s)'};
measLabels = {'\xi (m)','z (m)','\theta dot (rad/s)', '\xi ddot (m/s^2)' };           

uNomHist = Tnom*ones(length(tvec),2);
uNomHist(:,1) = uNomHist(:,1) + 0;

z0 = 20;% 15; %7.5;%  %nominal altitude
theta0 = 0* pi /180; %90 * pi/180; %nominal pitch angle
xi0 = 0; %nominal hoz position
xidot0 = 0; %nominal hoz velocity
zdot0 = 0; %nominal vert velocity
thetadot0 = 0; %nominal pitch rate
xNom0 = [xi0,xidot0,z0,zdot0,theta0,thetadot0]; 
perturb_x0 = [0; 0.2; 0; 0; 0; 0.001]; %small perturbation in IC
x0 = xNom0 + perturb_x0';

procNoiseOn = 1;
Qtilde = diag([1e-2 1e-2 1e-2]); %%nominal nonlinear DT AWGN covars
Stilde = chol(Qtilde);
wtilde0 = procNoiseOn * (Stilde*(randn(length(tvec),3))')'; 

xhist_nl = zeros(6,length(tvec));
xtt = x0;
xhist_nl(:,1) = x0;

measNoiseOn = 1;
Rtilde = diag([5^2 0.5^2 0.005^2 (0.015)^2]);
Ttilde = chol(Rtilde);
vtilde0 = measNoiseOn *  (Ttilde*(randn(length(tvec),4))')';

yhist_nl = zeros(4,length(tvec));
yhist_nl(:,1) = nan; %ignore first entry for x0

for tt=1:length(tvec)-1
    %%advance state through nonlinear dynamics 
    utt = uNomHist(tt,:);
    wtt = wtilde0(tt,:); 
    [ttt, xttp1] = ode45(@(t,x) SimpleSkyCrane_NLDynamics(t,x,utt,wtt,vehparams),[0 dt],xtt);
    %disp(length(xttp1));
    xhist_nl(:,tt+1) = xttp1(end,:)';
    
    %%simulate nonlinear measurements
    vttp1 = vtilde0(tt,:)';
    uttp1 = uNomHist(tt+1,:); %%need to look at ctrl input at ttp1
    yttp1 = SimpleSkyCrane_NLMeasurements(xttp1',uttp1,vehparams) + vttp1;
    yhist_nl(:,tt+1) = yttp1; 
    
    xtt = xhist_nl(:,tt+1); %iterate for next time
end

figure(10),
title('States vs. Time, Full Nonlinear Dynamics Simulation')

for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot(tvec,xhist_nl(',num2str(ii),',:))'])
ylabel(stateLabels{ii});
xlabel('Time (secs)')
end

%%validate data plots
%%full NL data sim
figure(70)
title('Full Nonlinear Model Data Simulation')
for ii=1:4
eval(['subplot(41',num2str(ii),')'])
eval(['plot(tvec,yhist_nl(',num2str(ii),',:))'])
ylabel(measLabels{ii});
xlabel('Time (secs)')
end


%% 2. Linearized model sim about nominal hover and data generation

%%Test jacobians and LTI controllability/observability/eigenvalues for nominal hover
[XJac,UJac,WJac] = SimpleSkyCrane_DynJacobians(x0,uNomHist(1,:),wtilde0(1,:),vehparams);

%%CT LTI Controllability: CT is continious time
bigC = ctrb(XJac,UJac);
rankbigC = rank(bigC);

%%CT LTI Observability:
H = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 0 1;
     XJac(2,:)];
bigO = obsv(XJac,H);
rankbigO = rank(bigO);

%%CT LTI eigenvalues/stability
EigCTvalsnom = eig(XJac);

%%DT LTI eigenvalues/stability, DT is discrete time
Fnom = expm(XJac*dt);
format long
EigDTvalsnom = eig(Fnom); %show in format long -- 2 are very close to 1

%repmat(A,n) returns an array containing n copies of A in the row and column dimensions. The size of B is size(A)*n when A is a matrix
%%%simulate linearized dynamics perturbations about nominal state
xNomHist = repmat(xNom0',[1 length(tvec)]); %unperturbed/undisturbed nominal state trajectory about equlibrium
yNom = zeros(4,length(tvec));
yNom(:,1) = nan; %ignore xnom at k=0;

%%define initial state perturbation vector
deltax0 = perturb_x0;
deltaxtt = deltax0;
deltaxHist = zeros(6,length(tvec));
deltaxHist(:,1) = deltax0;

%%define input perturbation
deltauHist = zeros(2,length(tvec));
%arrays to store measurements
deltayHist = zeros(4,length(tvec));
yNomHist = zeros(4,length(tvec));

%%Compute DT Jacobians 
%%(CT Jacobians here are constant/LTI for constant nominal u[t] and xNom at static hover):    
ABaugtilde = [XJac, UJac; 
        zeros(2,8)];     
eMaug = expm(ABaugtilde*dt);
Ftildett = eMaug(1:6,1:6);
Gtildett = eMaug(1:6,7:end);
%%use Euler discretization method to compute Omegatt for small DeltaT
Gamma = zeros(6,3);
Gamma(2,1) = 1;
Gamma(4,2) = 1;
Gamma(6,3) = 1;
Omegatildett = Gamma*dt; 
%%Get DT measurement Jacobians for constant nominal state and control inputs 
%%(note: meas y(k+1) depends on u(k+1))
[Ctilde,Dtilde] = SimpleSkyCrane_MeasJacobians(xNom0,uNomHist(1,:),vehparams); 
Htildett = Ctilde;
Mtildett = Dtilde; 

for tt=1:length(tvec)-1
    xNomtt = xNomHist(:,tt); 
    utt = uNomHist(tt,:)';
    deltautt = deltauHist(:,tt); 
    %%Compute DT perturbation updates
    %%%need to reorganize DT w input for linear model
    wtt = wtilde0(tt,:)'; 
    deltaxttp1 = Ftildett*deltaxtt + Gtildett*deltautt + Omegatildett*wtt;
    deltaxHist(:,tt+1) = deltaxttp1;
    
    %%Simulate nominal and perturbed measurements
    %%%nominal:
    yNomHist(:,tt+1) = SimpleSkyCrane_NLMeasurements(xNomtt,utt,vehparams);    
    
    %%%perturbations:
    vttp1 = vtilde0(tt,:)';
    deltauttp1 = deltauHist(:,tt+1); %%need to look at ctrl input at ttp1
    deltayttp1 = Htildett*deltaxttp1 + Mtildett*deltauttp1 + vttp1;
    deltayHist(:,tt+1) = deltayttp1; 
    
    deltaxtt = deltaxttp1; %iterate for next time
end
%%Get total approximate state trajectory
xTotApproxHist = xNomHist + deltaxHist;
yTotApproxHist = yNomHist + deltayHist; 

figure(20),
title('States vs. Time, Linearized Approximate Dynamics Simulation')
for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot(tvec,xTotApproxHist(',num2str(ii),',:))'])
eval(['ylabel(''',stateLabels{ii},''')'])
xlabel('Time (secs)')
axis tight
end

figure(21),
title('Linearized approx perturbations vs. Time')
for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot(tvec,deltaxHist(',num2str(ii),',:))'])
dellabel = ['\delta',stateLabels{ii}];
ylabel(dellabel)
xlabel('Time (secs)')
axis tight
end

figure(80) 
title('Approximate Linearized Model Data Simulation')
for ii=1:4
eval(['subplot(41',num2str(ii),')'])
eval(['plot(tvec,yTotApproxHist(',num2str(ii),',:))'])
ylabel(measLabels{ii});
xlabel('Time (secs)')
axis tight
end


%% Implement LQR controller on full NL system for KF obs data gen
load SkyCraneLinearFullStateLQRController.mat Klin

rng(863)
%%generate noise inputs
wtilde2 = (Stilde*(randn(length(tvec),3))')'; 
vtilde2 = (Ttilde*(randn(length(tvec),4))')';

xhistCL_nlBASE = zeros(6,length(tvec)); 
P0_BASE = diag([2 1 1 1 0.01 0.0005]); %%initial covar on perturbation
xtt = x0' + mvnrnd(zeros(1,6),P0_BASE)';
xhistCL_nlBASE(:,1) = xtt;
xref = xNom0'; %%LQR is going to try to maintain this state
uhistCL_nlBASE = zeros(2,length(tvec));
yhistCL_nlBASE = zeros(4,length(tvec)); 
yhistCL_nlBASE(:,1) = nan; %ignore very first time stamp
for tt=1:length(tvec)-1
    uttCL = uNomHist(tt,:) - (Klin*(xtt-xref))'; %%apply linear state feedback ctrl law
    wtt = wtilde2(tt,:); 
    [~, xttp1] = ode45(@(t,x) SimpleSkyCrane_NLDynamics(t,x,uttCL,wtt,vehparams),[0 dt],xtt);
    xhistCL_nlBASE(:,tt+1) = xttp1(end,:)';
    xtt = xhistCL_nlBASE(:,tt+1); %iterate for next time step
    uhistCL_nlBASE(:,tt)=uttCL;   
    
    %%simulate nonlinear measurements
    vttp1 = vtilde2(tt,:)';
    uttp1CL =  uNomHist(tt+1,:) - (Klin*(xhistCL_nlBASE(:,tt+1)-xref))'; %%need feedback ctrl input at time k+1
    yttp1 = SimpleSkyCrane_NLMeasurements(xhistCL_nlBASE(:,tt+1),uttp1CL,vehparams) + vttp1;
    yhistCL_nlBASE(:,tt+1) = yttp1;   
    
end
uhistCL_nlBASE(:,end) = Tnom; 
deltahist_nlBASE = uhistCL_nlBASE - uNomHist'; %%****STORE THIS

figure(90),
for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot(tvec,xhistCL_nlBASE(',num2str(ii),',:))'])
ylabel(stateLabels{ii});
xlabel('Time (secs)')
end
title('BASE Closed loop LQR Nonlinear State Response')


%%show total actual base thrust
figure(91), hold on
plot(tvec,uhistCL_nlBASE(1,:),'r')
plot(tvec,uhistCL_nlBASE(2,:),'b')
title('BASE LQR total actual thrusts T_1 and T_2 (N)')
legend('T_1', 'T_2')


%%show measurements to be provided -->> ***STORE THESE
figure(92),
for ii=1:4
eval(['subplot(41',num2str(ii),')'])
eval(['plot(tvec,yhistCL_nlBASE(',num2str(ii),',:))'])
xlabel('Time, sec')
ylabel(measLabels{ii})
axis tight
end
title('Nonlinear Measurements for Closed Loop LQR Control')

%% save yNL and LQR control law for KF implementation on assignment
%%... save tvec,yhistCL_nlBASE, Klin, Qtilde->Q, Rtilde->R, P0_BASE to provide... 
ydata = yhistCL_nlBASE;
Qtrue = Qtilde;
Rtrue = Rtilde;
uhist = uhistCL_nlBASE; 
save skycrane_finalproj_KFdata.mat -mat tvec ydata Klin measLabels Qtrue Rtrue uhist
%%save ground truth xhist data:
xhist = xhistCL_nlBASE; 
save skycrane_finalproj_KFtrueStateHist.mat -mat tvec xhist stateLabels 