% bias_param.m
%
% Example of bias estimator with simple moving mass
clear

% start in continuous time, use MKS units
% states: x1=pos, x2=vel, w1=bias
% calculate ss matricies from parameters, then simulate as ss models.

M1 = 10;   % mass kg
kv = 0.2;  % viscous damping
kf = 2.0;  % force constant

w = 2.0;   % external bias
ref = 10;  % reference position
Nx = [1; 0];

%% c.t. plant model w/o bias
% two states: pos, vel 
A = [0  1
     0 -kv/M1];
B = [0 
    kf];
C = [1 0];
D = 0;
sys_plant=ss(A,B,C,D);

%% discrete plant
% two states: pos, vel
Ts= 0.10;
sysd_plant= c2d(sys_plant,Ts);
Ad = sysd_plant.A;
Bd = sysd_plant.B;
Cd = sysd_plant.C;
Dd = sysd_plant.D;

%% LQR gain for feedback, two states
Qf = 1.0;
Rf = 1.0;
[Kgain] = lqr(sysd_plant,Qf,Rf);

%% c.t. estimator model augmented w/ bias 
% 3 states: pos, vel, bias
Aw = [A B                                  % <<=======
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
% computinh eig values of A-LC so,
% use plant 
Qe = 1.0;
Re = 1.0;
% Ae = 
% [Lp] = lqr(sysd_plant,Qe,Re);

P= [0.85+0.15i, 0.85-0.15i, 0.60]
Lp = place(Aob',Cob',P)'


