clear all; clc;
s = tf('s');

%% Params
g = 9.81;           % m/s^2  - gravitational constant

Mp = 1.51;          % kg  - robot body (plant)
lp = 0.08;         % m   - distance from axel to center of top mass
l = lp;
m = 0.33;           % kg  - robot body (plant)
M = 1.180;
Jp = 6.533e-5;      % Moment of inertia (PITCH) Point mass estimation
r_wheel = 0.04;     % m - radius of wheel
Jw = 1e-8;          % moment of inertia of wheel (small)
bg = 1e-3;          % friction from ground

% Motor
n = 18.75;          % pololu motor pinion gear ratio
Km = 1/0.062;       % from datasheet
bw = 1e-3;
l_axel = 0.2359;    % m - distance between the wheels
Jp_yaw = 9.0372e-5; % Moment of inertia (YAW)

% Microcontroller   
DC = 0.85;
Clock_f = 64e6;
Ctime = 0.025;
CF = 1/Ctime;
dt = 1/CF;

IMU_tread = 18e-3;
Encoder_tread = 3e-3;
Ktj = pi/180;
Kjt = 180/pi;

%% System State Space Matrix

% states: x, x*, t, t*
% with joint damping:
A = [ 0, 1, 0, 0;
    0,  -bg/M, (g*m)/M, -bw/(lp*M);
    0, 0, 0, 1;
    0,  -bg/(lp*M), (g*(M+m))/(lp*M), -bw*(M+m)/(lp^2*M*m); ];

% without joint damping:
% A = [0 1 0 0;
%     0 -bg/M m*g/M 0;
%     0 0 0 1;
%     0 -bg/(M*lp) (m+M)*g/(M*lp) 0];

B = [0; 1/M; 0; 1/(lp*M)];  % input matrix
B = B ./ r_wheel;           % adjust input from force to wheel torque.
C = [ 1 0 0 0; 0 0 1 0;];   % output matrix
D = [0; 0;];
Cx = C(1,:);
C_xt = eye(4);
D_xt = [0;0;0;0];
Dx = 0;
Ct = C(2,:);
Dt = 0;
x0 = [0; 0; 5*pi/180; 0]; % init state 

xss = ss(A,B,Cx,Dx);
tss = ss(A,B,Ct,Dt);
xtss = ss(A,B,C_xt, D_xt);
Gx = tf(xss);
Gt = tf(tss); % theta/torque

% Yaw
% states: delta, delta*
Ay = [0 1;  0 0];
By = [0;  2*r_wheel/(Jw*l_axel);  ];
Cy = [1 0];
Dy = 0;
Y_SS=ss(Ay,By,Cy,Dy);

Ga = importGa();
Ye = importYe();

%% Gp
G_wheelspeed = Gx / r_wheel * s; % -> (Ymech'): rad/s /torque
Gp_x_char = n*Km*Ye*G_wheelspeed; % characteristic eq.
Gp_x = Gp_x_char/ (1+ Gp_x_char) * r_wheel/s;
Gp_t = n*Ye*Km*Gt;
T.Gp = Gp_t;
X.Gp = Gp_x;

%% Hs (sensor delay)
T.exe_time = IMU_tread;
T.wn = 2*pi/T.exe_time;
T.Hs = T.wn/(s+T.wn); % sensor delay

X.exe_time = Encoder_tread;
X.wn = 2*pi/X.exe_time;
X.Hs = X.wn/(s+X.wn); 

%% Hc
% ===== Theta ===== %
% FIR for sensor to compenstate for FDD noise
T.GHs = Ga*T.Gp*T.Hs;
T.syspoles = real(pole(minreal(T.GHs)));
T.negpoles = T.syspoles(T.syspoles < 0);
T.mdp_GHs = max(T.negpoles); % most dom stable sys pole
if T.mdp_GHs > 0
    T.ndp = abs(10*ceil(T.mdp_GHs));
else
    T.ndp = abs(10*floor(T.mdp_GHs)); 
end
T.N_feedback = CF/abs(T.ndp);
T.Noh = DC + 0.5; % controller overhead delay multiple
T.iir_Nf = abs(CF/T.ndp) - DC - 0.5; % max filter delay for non-dom FB dynamics
T.iir_beta = T.iir_Nf/(T.iir_Nf+1);
T.iir_tau = -dt/log(T.iir_beta);
T.iir_N = T.N_feedback + T.Noh; % filter + overhead delay
T.fir_beta = T.iir_Nf/(T.iir_Nf + 1);
[T.fir_W, T.fir_Nf, T.fir_NC] = getFIR(T.fir_beta);
T.fir_N = T.iir_Nf + 0.5 + DC + 0.5; % FIR Filter Coef
T.wc = CF/T.fir_N;
T.Hc = s*T.wc/(s+T.wc);
 
% ===== X ===== %
% no FDD, just a filter
X.GHs = Ga*X.Gp*X.Hs;
X.syspoles = real(pole(minreal(X.GHs)));
X.negpoles = X.syspoles(X.syspoles < 0);
X.mdp_GHs = max(X.negpoles); % most dom stable sys pole
if X.mdp_GHs > 0
    X.ndp = abs(10*ceil(X.mdp_GHs));
else
    X.ndp = abs(10*floor(X.mdp_GHs)); 
end
X.N_feedback = CF/abs(X.ndp);
X.Noh = DC + 0.5; % controller overhead delay multiple
X.iir_Nf = abs(CF/X.ndp); % max filter delay for non-dom FB dynamics
X.iir_beta = X.iir_Nf/(X.iir_Nf+1);
X.iir_tau = -dt/log(X.iir_beta);
X.iir_N = X.N_feedback + X.Noh; % filter + overhead delay
X.fir_beta = X.iir_Nf/(X.iir_Nf + 1);
[X.fir_W, X.fir_Nf, X.fir_NC] = getFIR(X.fir_beta);
X.fir_N = X.iir_Nf + 0.5 + DC + 0.5; % FIR Filter Coef
X.wc = CF/X.fir_N;
X.Hc = s*X.wc/(s+X.wc);

%% T GH 
T.G = 2*Ga*T.Gp;
T.H = T.Hc*T.Hs;
T.GH = T.G*T.H;

%% T Cont
T.zmax = max(real(pole(T.GH)));
T.wz = -T.zmax; % cancel out unstable pole

% IIR on Derivative path in controller
T.mcu_iir_tau =  T.iir_tau/2;
T.mcu_iir_beta = exp(-dt/T.mcu_iir_tau);
T.mcu_iir_nf = T.mcu_iir_beta/(1-T.iir_beta);
T.mcu_iir_n = T.mcu_iir_nf + T.Noh + 0.5;
T.mcu_iir_wp = CF/T.mcu_iir_n;
T.Dp = T.mcu_iir_wp/(s+T.mcu_iir_wp) * 1/s;

%% T Zeros
T.oltf1 = T.Dp * T.GH;
[T.gm, ~,~,~] = margin(T.oltf1);
T.K0 = T.gm;
T.oltf2 = T.K0* T.Dp * T.GH;
[~,~,T.wcg,~] = margin(T.oltf2);
T.wxo = T.wcg;
[T.Z, T.PM, T.D] = findZeros(T.K0, T.wxo, T.GH, T.Dp);
T.oltf3 = T.K0* T.D * T.GH;
T.cltf1 = T.D * T.G/(1+T.D * T.GH);

%% T Refining D
T.K1 = T.K0;
T.p = -T.mcu_iir_wp;
T.z1 = T.Z(1);
T.z2 = T.Z(2);
T.zsum = real(T.z1)*2;
T.z1z2 = real(T.z1)^2 + imag(T.z1)^2;

T.Kp = (1/T.p - (T.zsum)/(T.z1z2)) * T.K1;
T.Ki = T.K1;
T.Kd = (1/T.p^2 - (T.zsum-T.p)/(T.p*T.z1z2)) * T.K1;

T.D2 = T.Kp + T.Ki/s + T.Kd * -T.p*s/(s-T.p);
T.oltf4 = T.D2*T.GH;
T.cltf2 = T.D2*T.G/(1+T.D2*T.GH);

%% X GH

X.G = T.cltf2; % cascade
X.H = X.Hc*X.Hs;
X.GH = X.G*X.H;


X.zmax = max(real(pole(X.GH)));
X.wz = -X.zmax;

% IIR on Derivative path in controller
X.mcu_iir_tau =  X.iir_tau/2;
X.mcu_iir_beta = exp(-dt/X.mcu_iir_tau);
X.mcu_iir_nf = X.mcu_iir_beta/(1-X.iir_beta);
X.mcu_iir_n = X.mcu_iir_nf + X.Noh + 0.5;
X.mcu_iir_wp = CF/X.mcu_iir_n;
X.Dp = X.mcu_iir_wp/(s+X.mcu_iir_wp) * 1/s;

%% Zeros

X.oltf1 = X.Dp * X.GH;
[X.gm, ~,~,~] = margin(X.oltf1);
X.K0 = X.gm;
X.oltf2 = X.K0* X.Dp * X.GH;
[~,~,X.wcg,~] = margin(X.oltf2);
X.wxo = X.wcg;
[X.Z, X.PM, X.D] = findZeros(X.K0, X.wxo, X.GH, X.Dp);
X.cltf1 = X.D * X.G/(1+X.D * X.GH);

X.K1 = X.K0;

X.p = -X.mcu_iir_wp;
X.z1 = X.Z(1);
X.z2 = X.Z(2);
X.zsum = real(X.z1)*2;
X.z1z2 = real(X.z1)^2 + imag(X.z1)^2;

X.Kp = (1/X.p - (X.zsum)/(X.z1z2)) * X.K1;
X.Ki = X.K1;
X.Kd = (1/X.p^2 - (X.zsum-X.p)/(X.p*X.z1z2)) * X.K1;

X.D2 = X.Kp + X.Ki/s + X.Kd * -X.p*s/(s-X.p);
X.oltf4 = X.D2*X.GH;
X.cltf2 = X.D2*X.G/(1+X.D2*X.GH);

%% Ktune
K.Kpx = X.Kp;
K.Kix = X.Ki;
K.Kdx = X.Kd;

K.Kpt = T.Kp;
K.Kit = T.Ki;
K.Kdt = T.Kd;

KtuneCascade(K, X, T, 0.5, 2)