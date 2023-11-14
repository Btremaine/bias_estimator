% bias_param.m
%
% Example of bias estimator with simple moving mass
clear

% start in continuous time, use MKS units
% states: x1=pos, x2=vel, w1=bias
% calculate ss matricies from parameters, then simulate as ss models.

M1 = 5;    % mass kg
kv = 0.2;  % viscous damping
kf = 2.0;  % force constant

%% c.t. plant model without bias
% two states: pos, vel
% x2d = -(kv/M1) * x2 + (kf/M) * u
% x1d = x2
A = [0  1 
     0 -kv/M1];
B = [0 
    kf]/M1;
C = [1 0];
D = 0;
sys_plant=ss(A,B,C,D);

%% discrete plant without bias
% two states: pos, vel
Ts= 0.10;
sysd_plant= c2d(sys_plant,Ts);
Ad = sysd_plant.A;
Bd = sysd_plant.B;
Cd = sysd_plant.C;
Dd = sysd_plant.D;

%% LQR gain for feedback, two states
Qf = [1. 0
      0  0.1];
Rf = 1.0;
[Kgain] = lqr(sysd_plant,Qf,Rf);

%% c.t. estimator model augmented with bias 
%%
% 3 states: pos, vel, bias
Aw = [A B                                  % <<======= check this !!!!!
      0 zeros(1,2)]; % dw/dt = 0
Bw = [B ; 0];
Cw = [C 0];
Dw = 0;
na = size(Aw,1);     % # states in aug plant
sys_aug = ss(Aw,Bw,Cw,Dw);

%% discrete estimator : used in Simulink
%
sys_obsv = c2d(sys_aug,Ts);
Aob= sys_obsv.A;
Bob= sys_obsv.B;
Cob= sys_obsv.C;
Dob= sys_obsv.D;

%% controllability and observability
co = rank(ctrb(Ad,Bd));
ob= rank(obsv(Ad,Cd));
disp("plant")
fprintf('co: %i\n', co)
fprintf('ob: %i\n', ob)

disp("Augmentented with bias") % Note, controllable
co = rank(ctrb(Aob,Bob));      % but not observable
ob = rank(obsv(Aob,Cob));
fprintf('co: %i\n', co)
fprintf('ob: %i\n', ob)

%% LQR gain for estimator Lp
% computing eig values of A-LC
% using observer
%
P= [0.85+0.15i, 0.85-0.15i, 0.40]
Lp = place(Aob',Cob',P)'

% Ref-feedforward
Nx = [1.0;0];
H = [1.0 0];
% set ref step
ref = 10.0;

%% Saturation Limits 
scenario = 2
if  scenario == 1
  SAT1=   5.0; % level at which "actuator/driver" saturates
  SAT2=   5.0; % level at whicrich "processor" output saturates
  SAT3=   5.0; % level at which  fixed-point-math saturates
elseif scenario == 2 
 SAT1=2.70; % Driver/actuator saturation
 SAT2=2.20; % DAC/PWM saturation
 SAT3=2.05; % FP saturation
else
  print('pick a valid scenario, 1 or 2')
%% criteria S2 >= S1, and S2 >= S3

end


