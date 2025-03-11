%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assignment 1
% ======================================

% Initialize
% Clean up workspace
clear all; clc;
SN = 48082044;
A = 14; B = 18; C = 10; D = 18; E = 12; F = 10; G = 14; H = 14;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Params

% table 1 gripper
js = A/5 * 1e-7; % g-cm^2 -> kg-m^2
ms = B/4 * 1e-3;
bs = C/3;
ns = D/(2*E) * 2*pi*100; % turn/cm -> rad/m
jf = F/3 * 1e-7;
bf = G * 1e-3;
nf = H * (pi/180) * 100; % deg/cm -> rad/m
bt = A * 1e-3;
kt = B*C * 1e-3;

bl = D/5;
kl = E*F;
l6 = (G+H)*4 * 1e-3;
nt = l6; % from geometry

% table 2
P1 = A*7;
P2 = B*800;
P3 = C*8;
P4 = D*700; 
P5 = E*600; 
P6 = F*50; 
P7 = G*500; 
P8 = H*5; 

% table 3 motor
rw = A/2;
lw = B*30 * 1e-6;
km = C * 1e-3;
jr = D/15 * 1e-7;
br = E/30 * 1e-6;
mm = F+G;

% table 4
CF = A*B*5 * 2*pi; % Hz -> rad
DC = (C+D+E+F)/100; % percentage -> decimal

% table 5
MagRes = 0.1;
AngRes = 1 * pi/180; % deg -> rad
TargPM = 40 * pi/180; % deg -> rad
OSu = 60 /100; % percent -> dec
Ts = 75 /100;
% Tr < final Ts (sec)
Ess = 0;

%% Q1

Q1.yp = 17.6;
Q1.fv = 14.1;

Q1.Tr = 0.74 *1e-3;
Q1.Tp = 1.61 *1.01 *1e-3;
Q1.Ts = 4.8 *0.98 *1e-3;
Q1.OSy = (Q1.yp/Q1.fv - 1) * 100;

%% Q2

Q2.Kdc = Q1.fv;
Q2.zeta = -1*log(Q1.OSy/100) / sqrt(pi^2 + (log(Q1.OSy/100)^2));
Q2.beta = sqrt(1-Q2.zeta^2);
Q2.wnp = pi/(Q1.Tp * Q2.beta);
Q2.Ga = Q2.Kdc * tf(Q2.wnp^2, [1, 2*Q2.zeta*Q2.wnp, Q2.wnp^2]);

%% Q3
s = tf('s');
G1 = P1 / (s + P2);
G2 = P3 / (s + P4);
G3 = 10^5 / (s + P5);
G4 = P6 / (s + P7);
H1 = 4 / (s + P8);

Q3.tfden = 1 + H1*G2*G3/(1 + G3*G4*H1);
Q3.tfnum = (G2*G1 + G2*G3) / (1+G3*G4*H1);
Q3.Hs = Q3.tfnum / Q3.tfden; % mv/deg 
Q3.Ks = dcgain(Q3.Hs) / 1000; % V/deg
Q3.Ds = Q3.Hs / dcgain(Q3.Hs) ;

%% Q4
Q4.Ye = 1/(s*lw+rw);

%% Q5

jx = jr + js + (mm+ms)/ns^2 + 3*jf*(nf/ns)^2;
bx = br + bs/ns^2 + 3*bf*(nf/ns)^2;

b1 = 3*bt*(nf/ns)^2;
b2 = 3*bl*(nf*nt/ns)^2;
k1 = 3*kt*(nf/ns)^2;
k2 = 3*kl*(nf*nt/ns)^2;

ykb1 = k1/s + b1;
ykb2 = k2/s + b2;
ykb = (ykb1*ykb2)/(ykb1+ykb2);
yj = jx*s;
yb = bx;
yt = yj+yb+ykb;

Q5.Ym = 1/yt;
%% Q6
Q6.taug = (mm)*9.81/ns; % Nm

%% Q7
Q7.Gtau = (km*Q4.Ye)/(1+km^2*Q4.Ye*Q5.Ym);

%% Q8
Q8.Gq = Q5.Ym/s * Q7.Gtau * 180/pi;

%% Q9
kt1 = kt/nt^2;
bt1 = bt/nt^2;
ykt1 = kt1/s;
ybt1 = bt1;
ykl = kl/s;
ybl = bl;
yt1 = ykt1 + ybt1;
yl = ykl + ybl;
ytl = yt1*yl/(yt1+yl);

Q9.Gf = (nf*nt/ns)*ytl*Q8.Gq*pi/180 * s;

%% Q10

% 1/s * Q4.Ye/(1+km^2 * Q5.Ym * Q4.Ye)
Q10.iw = dcgain(Q7.Gtau/km);%(A) Scalar
Q10.taur = Q10.iw * km;%(Nm) Scalar
Q10.fs = Q10.taur * ns;%(N) Scalar
Q10.tauf = Q10.fs/nf/3;%(Nm) Scalar
Q10.ft = Q10.tauf/nt;%(N) Scalar

qr_ss = dcgain(Q8.Gq); % deg
qf_ss = qr_ss * nf/ns;

qr_ss_rad = qr_ss * pi/180; % rad
qf_ss_rad = qr_ss_rad * nf/ns;
dt_ss_rad = qr_ss_rad *nf*nt/ns;

Q10.Ktl = Q10.tauf /qf_ss_rad;%(Nm) Scalar

Q10.qf = qf_ss;%(deg) Scalar

Q10.ds = qr_ss_rad/ns ; %(m) Scalar


Q10.qr = qr_ss;%(deg) Scalar
Q10.vs = dcgain(Q3.Hs/1000 * Q8.Gq); %(V) Scalar