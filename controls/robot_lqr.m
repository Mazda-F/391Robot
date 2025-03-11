clear all; clc;
% elec 391 controls
s = tf('s');

%% PARAMS
g = 9.81; % m/s^2  - gravitational constant

Mp = 1.51; % kg  - robot body (plant)
lp = 0.055; % m   - distance from axel to center of mass
m = 0.33; % kg  - robot body (plant)
M = 1.180;
Jp = 6.533e-5; % Moment of inertia (PITCH) Point mass estimation
r_wheel = 0.04; % m - radius of wheel
Jw = 1e-8; % moment of inertia of wheel (small)
bg = 1e-3; % friction from ground

% Motor
n = 18.75; % pololu motor pinion gear ratio
Km = 0.212207;       % from datasheet
l_axel = 0.2359; % m - distance between the wheels
Jp_yaw = 9.0372e-5; % Moment of inertia (YAW)
% Microcontroller   
DC = 0.3;
Clock_f = 64e6;
Ctime = 0.025;
CF = 1/Ctime;
dt = 1/CF;

IMU_tread = 18e-3;
Encoder_tread = 3e-3;
Ktj = pi/180;
Kjt = 180/pi;

%% ====== System ====== %%
%% State space matrix
% Pitch and Speed

% A = [ 0, 1, 0, 0;
%     0,  -bg/M, (g*m)/M, -bg/(lp*M);
%     0, 0, 0, 1;
%     0,  -bg/(lp*M), (g*(M+m))/(lp*M), -(bg*M+bg*m)/(lp^2*M*m);
% ];

A = [ 0, 1, 0, 0;
    0,  -bg/M, (g*m)/M, 0;
    0, 0, 0, 1;
    0,  -bg/(lp*M), (g*(M+m))/(lp*M), 0;
];

B = [0;0; 1/M; 1/(lp*M)]./ r_wheel;

C = [   1 0 0 0;
        0 0 1 0;];
Cx = C(1,:);

D = [0; 0;];
Dx = D(1,:);
x0 = [0; 0; 5*pi/180; 0];

XP_SS=ss(A,B,C,D);
xss = ss(A,B,C(1,:),D(1,:));

% Yaw Model
Ay = [0 1;
    0 0];
By = [0;
    2*r_wheel/(Jw*l_axel);
    ];
Cy = [1 0];
Dy = 0;
Y_SS=ss(Ay,By,Cy,Dy);

Cx = C(1,:);
C_xt = eye(4);
Dx = 0;
D_xt = [0;0;0;0];
Ct = C(2,:);
Dt = 0;
xss = ss(A,B,Cx,Dx);
tss = ss(A,B,Ct,Dt);
xtss = ss(A,B,C_xt, D_xt);
Gx = tf(xss);
Gt = tf(tss); % theta/torque

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

%% Controller

Q = [
1 0 0 0;
0 1 0 0;
0 0 10 0;
0 0 0 100;
];

R = 200;

K = lqr(A,B,Q,R)


