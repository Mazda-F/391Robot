clear all; clc;
s = tf('s');
%%
% Strategy: Tune the inner most loop with outer loops OPEN, and move
% outwards gradually

%% Params
g = 9.81;               % m/s^2  - gravitational constant
Mp = 1.51;              % kg  - robot body (plant)
Mw = 0.0005;            % kg  - mass of wheel
% l = 0.14174;            % m   - distance from axel to center of pendulum mass
l = 0.15;
mtot = 1.63;
M = 1.285;              % kg  - mass of lower chassis
m = mtot-M;               % kg  - robot upper load (pendulum)
R = 0.04;               % m - radius of wheel
J = 6.533e-5;           % Moment of inertia of entire robot
Jp = m*l^2;             % Moment of inertia of pendulum (point mass approx)
Jm = J - Jp;            % Moment of inertia of chassis
Jw = 1/2*Mw*R^2;        % Moment of inertia of wheel (small, disk approx)
Jp_delta = 9.0372e-5;   % Moment of inertia (YAW)
bg = 0.5;              % friction from ground

% Motor
n = 18.75;              % pololu motor pinion gear ratio
% Km = 0.212207;          % from datasheet
Km = 16.129/1000;
bw = 1e-3;
l_axel = 0.2359;        % m - distance between the wheels

% Microcontroller   
CF = 560; % Hz
dt = 1/CF;

IMU_tread = 0.1*dt; % IMU sensor delay
Encoder_tread = 0.5*dt; % Encoder sensor delay

DC = (IMU_tread + Encoder_tread)/dt;

%% System State Space Matrix

% states: x, x*, t, t*
% with joint damping:
% A = [ 0, 1, 0, 0;
%     0,  -bg/M, (g*m)/M, -bw/(l*M);
%     0, 0, 0, 1;
%     0,  -bg/(l*M), (g*(M+m))/(l*M), -bw*(M+m)/(l^2*M*m); ];

% without joint damping:
A = [0 1 0 0;
    0 -bg/M m*g/M 0;
    0 0 0 1;
    0 -bg/(M*l) (m+M)*g/(M*l) 0];

% A = [0 1 0 0;
%     0 0 m*g/M 0;
%     0 0 0 1;
%     0 0 (m+M)*g/(M*l) 0];

B = [0; 1/M; 0; 1/(l*M)] / R;  % input matrix.
C = eye(4);
D = [0; 0; 0; 0;];



xss = ss(A,B,C(1,:),0);
xdot_ss = ss(A,B,C(2,:),0);
tss = ss(A,B,C(3,:),0);
tdot_ss = ss(A,B,C(4,:),0);

G_x = tf(xss);
G_xdot = tf(xdot_ss);
G_t = tf(tss); % theta/torque
G_tdot = tf(tdot_ss);

% Yaw
% states: delta, delta*
Ay = [0 1;  0 0];
By = [0;  2*R/(Jw*l_axel);  ];
Cy = [1 0];
Dy = 0;
Y_SS=ss(Ay,By,Cy,Dy);

Ga = importGa();
Ye = importYe();

%% Theta Gp
G_motorspeed = G_xdot/R; % to find back emf for feedback path
T.Gp1 = G_t * 2 * n*Km*Ye/(1+ 2 * (n*Km)^2*Ye*G_motorspeed); % initial 2dof rep.


%% Hs (sensor delay)
T.exe_time = IMU_tread;
T.wn = 2*pi/T.exe_time;
T.Hs = T.wn/(s+T.wn); % sensor delay

X.exe_time = Encoder_tread;
X.wn = 2*pi/X.exe_time;
X.Hs = X.wn/(s+X.wn); 

%% Hc
% ===== Theta / IMU ===== %
% FIR for sensor to compenstate for FDD noise
T.GHs = Ga*T.Gp1*T.Hs;
T.syspoles = real(pole(minreal(T.GHs)));
T.mdp_GHs = max(T.syspoles); % most dom stable sys pole
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
 

%% A.1 Theta OLTF
T.G = Ga*T.Gp1*3.3/255;
T.H = T.Hc*T.Hs;
T.GH = T.G*T.H;
T.zmax = max(real(pole(T.GH)));
T.wz = -T.zmax; % cancel out unstable pole

% IIR on Derivative path in controller
T.mcu_iir_tau =  T.iir_tau/2;
T.mcu_iir_beta = exp(-dt/T.mcu_iir_tau);
T.mcu_iir_nf = T.mcu_iir_beta/(1-T.iir_beta);
T.mcu_iir_n = T.mcu_iir_nf + T.Noh + 0.5;
T.mcu_iir_wp = CF/T.mcu_iir_n;
T.Dp = T.mcu_iir_wp/(s+T.mcu_iir_wp) * 1/s;

% PID
T.oltf1 = T.Dp * T.GH;
[T.gm, ~,~,~] = margin(T.oltf1);
T.K0 = T.gm;
T.oltf2 = T.K0* T.Dp * T.GH;
[~,~,T.wcg,~] = margin(T.oltf2);
T.wxo = T.wcg;
[T.Z, T.PM, T.D] = findZeros(T.K0, T.wxo, T.GH, T.Dp);
T.oltf3 = T.K0* T.D * T.GH;
T.cltf1 = T.D * T.G/(1+T.D * T.GH);
T.K1 = T.K0;
%% A.2 Theta PID Init
T.p = -T.mcu_iir_wp;
T.z1 = T.Z(1);
T.z2 = T.Z(2);
T.zsum = real(T.z1)*2;
T.z1z2 = real(T.z1)^2 + imag(T.z1)^2;

T.Kp = (1/T.p - (T.zsum)/(T.z1z2)) * T.K1;
T.Ki = T.K1;
T.Kd = (1/T.p^2 - (T.zsum-T.p)/(T.p*T.z1z2)) * T.K1;
%% A.3 Theta Untuned CLTF

KT.Kp = T.Kp;
KT.Ki = T.Ki;
KT.Kd = T.Kd;
T.K=[1, T.Kp, T.Ki, T.Kd]; % Pre-Ktune Tvalues
Ktune(KT, T.G, T.H, T.p, 1, 5)
%% A.4 Theta Tuned CLTF

T.Kp = 319;
T.Ki = 1371;
T.Kd = 31.6;

T.Kp = 320;
T.Ki = 1200;
T.Kd = 30;

T.D = T.Kp + T.Ki/s + T.Kd * -T.p*s/(s-T.p);
T.cltf = T.D*T.G/(1+T.D*T.GH);

%% B.0 X System
% ===== X / Encoders ===== %
X.Gp = 2 * n*Km*Ye*G_x / (1 + 2 *(n*Km)^2*Ye*G_x);
X.GHs = Ga*X.Gp*X.Hs;
X.syspoles = real(pole(minreal(X.GHs)));
X.mdp_GHs = max(X.syspoles); % most dom stable sys pole
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
%% B.1 X OLTF

X.G = T.D *Ga*X.Gp*3.3/255; % cascaded pid
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
% PID
X.oltf1 = X.Dp * T.D * X.GH; % using X.GH for observing X feedback
[X.gm, ~,~,~] = margin(X.oltf1);
X.K0 = X.gm;
X.oltf2 = X.K0* X.Dp * X.GH;
[~,~,X.wcg,~] = margin(X.oltf2);
X.wxo = X.wcg;
[X.Z, X.PM, X.D] = findZeros(X.K0, X.wxo, X.GH, X.Dp);
X.cltf1 = X.D * X.G/(1+X.D * X.GH);
X.K1 = X.K0;
%% B.2 X PID Init

X.p = -X.mcu_iir_wp;
X.z1 = X.Z(1);
X.z2 = X.Z(2);
X.zsum = real(X.z1)*2;
X.z1z2 = real(X.z1)^2 + imag(X.z1)^2;

X.Kp = (1/X.p - (X.zsum)/(X.z1z2)) * X.K1;
X.Ki = X.K1;
X.Kd = (1/X.p^2 - (X.zsum-X.p)/(X.p*X.z1z2)) * X.K1;
%% B.3 X Untuned CLTF
KX.Kp = X.Kp;
KX.Ki = X.Ki;

KX.Kd = X.Kd;
X.K=[1, X.Kp, X.Ki, X.Kd]; % Pre-Ktune Tvalues
Ktune(KX, X.G, X.H, X.p, 0.05, 5)
%% B.4 X Tuned CLTF

X.Kp = -0.065;
X.Ki = -0.00285;
X.Kd = -0.198;

X.D = X.Kp + X.Ki/s + X.Kd * -X.p*s/(s-X.p);
X.cltf = X.D*X.G/(1+X.D*X.GH);
step(X.cltf)

%% Cascaded X and Theta

XT.T = X.D * T.cltf;
XT.X = X.cltf;
step(XT.X)