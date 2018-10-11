%test_SimpleSkyCraneDynamicsCtrl.m
%%Run test simulation of Simple Sky Crane NL Dynamics with nominal specs
clc,clear,close all
rng(100)
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
    /(vehparams.m_b+vehparams.m_f); %0.9412, part 4, 6th equation
vehparams.Ieta = (1/12)*...
    (vehparams.m_b*((vehparams.w_b)^2 + (vehparams.h_b)^2) ...
    +vehparams.m_f*((vehparams.w_f)^2 + (vehparams.h_f)^2) );%part 3 4th equation
vehparams.Aside = vehparams.h_b*vehparams.d_b + vehparams.h_f*vehparams.d_f; %part 4, 7th equation
vehparams.Abot = vehparams.w_b*vehparams.d_b + vehparams.w_f*vehparams.d_f; %part 4, 8th equation


%% 1. simulate non-linear dynamics for nominal static hover
%%%Compute nominal thrusts for static hover equilibrium using 2 thrusters
%%(note: actual thrusters had thrust range of 400-3060 N for 4 thrusters)
Tnom = 0.5* vehparams.g * (vehparams.m_b + vehparams.m_f)/cos(vehparams.beta);%part4 last equation

dt = 0.1;
tvec = 0:dt:50;

stateLabels = {'\zeta (m)', '\zeta dot (m/s)', 'z (m)','z dot (m/s)',...
               '\theta (rads)', '\theta dot (rad/s)'}; %state
measLabels = {'\zeta (m)','z (m)','\theta dot (rad/s)', '\zeta ddot (m/s^2)' };  %measurement         

uNom = Tnom*ones(length(tvec),2); %input
uNom(:,1) = uNom(:,1) + 0;%??

z0 = 20;% 15; %7.5;%  %nominal altitude
theta0 = 0* pi /180; %90 * pi/180; %nominal pitch angle
zeta0 = 0; %nominal hoz position
zetadot0 = 0; %nominal hoz velocity
zdot0 = 0; %nominal vert velocity
thetadot0 = 0; %nominal pitch rate
x0 = [zeta0,zetadot0,z0,zdot0,theta0,thetadot0]; 
perturb_x0 = [0; 0.2; 0; 0; 0; 0.001]; %small perturbation in IC
x0 = x0 + perturb_x0';

procNoiseOn = 0;
Qtilde = diag([1e-4 1e-1 1e-1]); %%nominal nonlinear DT AWGN covars
Stilde = chol(Qtilde);
wtilde0 = procNoiseOn*(Stilde*(randn(length(tvec),3))')'; %zeros(length(tvec),3);

xhist = zeros(6,length(tvec)+1);
xtt0 = x0;
xhist(:,1) = x0;
%For generating the ground truth, Xtt is the state. 
%See https://www.mathworks.com/help/matlab/ref/ode45.html about ODE45 (check the simple example in that page)
%The following for loop use ODE45 to solve the discrete differenctial equation
%Each time we use ODE45, we'll generate a bunch of State and Ttt in one dt!
%We just pick the last one of Xtt as the next time's(next dt) new state and
%compute again, that's how we generate 500(0 to 50, dt == 0.1) gorund
%truth
for tt=1:length(tvec)
    utt = uNom(tt,:);
    wtt = wtilde0(tt,:); 
    [Ttt, Xtt] = ode45(@(t,x) SimpleSkyCrane_NLDynamics(t,x,utt,wtt,vehparams),[0 dt],xtt0);
    xhist(:,tt+1) = Xtt(end,:)';
    xtt0 = xhist(:,tt+1); %iterate for next time
end
%After we generate the ground trurh we can plot the 6 states
figure(),
for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot([0,tvec],xhist(',num2str(ii),',:))'])
end

%% 2. Test jacobians for nominal hover
%we don't have the SimpleSkyCrane_Jacobians function, seems it should be SimpleSkyCrane_Dynjacobians
%Study Controbility and Obserbility in Advanced Linear System class, check
%https://nptel.ac.in/courses/108103008/29      to pick them up again.
%But why we need check the Controability and Observability matrix of Jacobian?
[XJac,UJac,WJac] = SimpleSkyCrane_DynJacobians(x0,uNom(1,:),wtilde0(1,:),vehparams);
%%Controllability:
bigC = ctrb(XJac,UJac);%If controllable, the controabolity matrix needs to have full rank
rankbigC = rank(bigC);

%%Observability:
% H = [1 0 0 0 0 0;
%      0 0 1 0 0 0;
%      0 0 0 0 1 0];
H = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 0 1;
     XJac(2,:)];
bigO = obsv(XJac,H);%similar as controllabitity matrix
rankbigO = rank(bigO);

D = zeros(size(H,1),2);
Hc = eye(6);
Dc = zeros(size(Hc,1),2);%Dc here is 0, Hc is eye, so the jacobian are being send to output with "perfect measurement" assumption
%%Convert to CT SS LTI model for controls stuff (assuming full state sensing)
OLSys = ss(XJac,UJac,Hc,Dc);% https://www.mathworks.com/help/control/ref/ss.html for ss

%% reduced order linear control design
% % %%Reduced order model for zeta and theta dynamics only
% % Ared_zeta = [XJac(1:2,1:2), XJac(1:2,5:6);
% %              XJac(5:6,1:2), XJac(5:6,5:6)]
% % Bred_zeta = [UJac(1:2,:);
% %              UJac(5:6,:)]
% % bigCredzeta = ctrb(Ared_zeta,Bred_zeta);
% % rank(bigCredzeta)

% %%Reduced order model for z and theta dynamics only
% Ared_z =    [XJac(3:4,3:4), XJac(3:4,5:6);
%              XJac(5:6,3:4), XJac(5:6,5:6)]
% Bred_z =    [UJac(3:4,:);
%              UJac(5:6,:)]
% bigCredz = ctrb(Ared_z,Bred_z); %%verify controllability
% rankbigredCredz = rank(bigCredz)
% %%Design LQR for joint altitude and theta hold
% %%%--Do quick lsim of reduced order z/theta system with process noises:
% Bred_zaug = [Bred_z, [WJac(3:4,2:3); WJac(5:6,2:3)] ]; %%augment for proc noise   
% OLredzaug = ss(Ared_z,Bred_zaug,eye(4),zeros(4,4));
% uredzaug = [zeros(length(tvec),2),wtilde0(:,2:3)]; %sim proc noise in
% figure()
% lsim(OLredzaug,uredzaug,tvec)
% 
% %%--design LQR control law
% OLredz = ss(Ared_z,Bred_z,eye(4),zeros(4,2));
% Qredz = diag([200 15 10000 15]);
% Rredz = diag([0.01 0.01]);
% [Kredz,~,~] = lqr(OLredz,Qredz,Rredz);
% 
% %%---implement CL linear reduced order system with LQR gain and process noise + full state feedback
% Ared_zCL = Ared_z - Bred_z*Kredz;
% Bred_zaugCL = Bred_zaug(:,3:4); %%include only process noise piece
% CLredzaug = ss(Ared_zCL, Bred_zaugCL, eye(4),zeros(4,2));
% wredzaugCL = wtilde0(:,2:3);
% figure()
% lsim(CLredzaug,wredzaugCL,tvec)
% %%show control effort (thrust deviations from nominal)
% [~,~,Xcl]=lsim(CLredzaug,wredzaugCL,tvec);
% uCLred = Kredz*Xcl';
% figure(), hold on
% plot(tvec,uCLred(1,:),'r')
% plot(tvec,uCLred(2,:),'b')
% title('Reduced Order LQR Linear Model Thruster Deviations \delta T_1 and \delta T_2 (N)')
% legend('\delta T_1', '\delta T_2')


%% full linear system control design
Alin = XJac;
Blin = UJac;
%%---LQR design....
%%%--Do quick lsim of full linearized system with process noises:
Blin_aug = [UJac, WJac]; %%augment for proc noise   
OLaug = ss(Alin,Blin_aug,eye(6),zeros(6,5));
u_aug = [zeros(length(tvec),2),wtilde0]; %sim proc noise in
figure()
lsim(OLaug,u_aug,tvec)

%%--design LQR control law
OLlin = ss(Alin,Blin,eye(6),zeros(6,2));
Qlin = diag([200 15 200 15 10000 15]);
Rlin = diag([0.01 0.01]);
[Klin,~,~] = lqr(OLlin,Qlin,Rlin);

%%---implement CL linear reduced order system with LQR gain and process noise + full state feedback
Alin_CL = Alin - Blin*Klin;
Blin_CL = WJac; %%include only process noise piece
CLlin = ss(Alin_CL, Blin_CL, eye(6), zeros(6,3));
wlinCL = wtilde0;
figure()
lsim(CLlin,wlinCL,tvec)
%%show control effort (thrust deviations from nominal)
[~,~,Xcl]=lsim(CLlin,wlinCL,tvec);
uCLlin = Klin*Xcl';
figure(), hold on
plot(tvec,uCLlin(1,:),'r')
plot(tvec,uCLlin(2,:),'b')
title('Full Linearized LQR Thruster Deviations \delta T_1 and \delta T_2 (N)')
legend('\delta T_1', '\delta T_2')

%% implement desired LQR control on full NL system
Knl = Klin; %[zeros(2,2),Kredz];
xhistCL = zeros(6,length(tvec)+1);
xtt0 = x0';
xhistCL(:,1) = x0;
xref = x0';
for tt=1:length(tvec)
    uttCL = uNom(tt,:) - (Knl*(xtt0-xref))'; %%should lqr gain K be computed for DT??
    wtt = wtilde0(tt,:); 
    [Ttt, Xtt] = ode45(@(t,x) SimpleSkyCrane_NLDynamics(t,x,uttCL,wtt,vehparams),[0 dt],xtt0);
    xhistCL(:,tt+1) = Xtt(end,:)';
    xtt0 = xhistCL(:,tt+1); %iterate for next time
end

figure(),
for ii=1:6
eval(['subplot(61',num2str(ii),')'])
eval(['plot([0,tvec],xhistCL(',num2str(ii),',:))'])
end
title('Closed loop LQR Nonlinear Response')
%%show total actual thrust
uNLCL = uNom' + Knl*xhistCL(:,1:end-1); 
figure(), hold on
plot(tvec,uNLCL(1,:),'r')
plot(tvec,uNLCL(2,:),'b')
title('Total actual thrusts T_1 and T_2 (N)')
legend('T_1', 'T_2')
