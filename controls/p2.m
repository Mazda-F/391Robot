%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project 2
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


%% Part 2 PARAMS

% table 4
CF = A*B*5; % Hz
DC = (C+D+E+F)/100; % percentage -> decimal


%% Q11

Q11.G = Q2.Ga * Q8.Gq * pi/180; %(rad/V) LTI Object
Q11.Ktj = ns/nf/l6;%(rad/deg) scalar
Q11.Kjt = l6*nf/ns;


%% Q12
dt = 1/CF;
Q12.Noh = DC + 0.5; % Duty cycle + half, where reading truly accurate

%% Q13
GHs = Q11.G * Q3.Hs;
p = pole(zpk(GHs));

Q13.wd = 1*1.02; %(rad/s) Scalar
wnd = ceil(Q13.wd)*10;
N_fb = CF/wnd;

% IIR
Q13.Nf = N_fb - DC - 0.5;%(pure) Scalar
Q13.beta = Q13.Nf/(Q13.Nf+1);%(pure) Scalar
Q13.tau = -dt/log(Q13.beta);%(s) Scalar
Q13.N = N_fb + Q12.Noh;% filter and overhead delay

%% Q14
% FIR

Q14.beta = Q13.N/(Q13.N+1);
Q14.beta_fir = Q13.Nf/(Q13.Nf+1);
% Q14.beta^250
% Q14.W(Q14.NC)/Q14.W(1)
Q14.NC = 250; %(pure) Scalar

% fir coefficients
wsum = 0;
for idx = 1:Q14.NC
    Q14.W(idx) = Q14.beta_fir^(idx-1);
    wsum = wsum + Q14.W(idx);
end
Q14.W = Q14.W/wsum;


%% Q15
Nf_new = 0;
for idx = 1:Q14.NC
    Nf_new = Nf_new + (idx-1)*Q14.W(idx);
end
Q15.N = Nf_new + Q12.Noh;
Q15.wp = CF/Q15.N;
Q15.Hc = Q15.wp/(s+Q15.wp) * pi/180 / Q3.Ks;%(rad/V) LTI Object;
H_temp = Q15.Hc*Q3.Hs;
Q15.H  = H_temp/dcgain(H_temp);%

%% Q16, compensate for FDD noise
iir_tau = Q13.tau/2;
iir_beta = exp(-dt/iir_tau);
iir_nf = iir_beta/(1-iir_beta);
iir_n = iir_nf + Q12.Noh + 0.5;
iir_wp = CF/iir_n;
Q16.Dp = iir_wp/(s+iir_wp) * 1/s;%(pure) LTI Object

oltf = Q16.Dp*Q11.G*Q15.H;
[Gm, ~, ~, ~] = margin(oltf);

Q16.K0 = Gm; %(Vs/m) Scalar

oltf2 = Q16.K0 * Q16.Dp*Q11.G*Q15.H; 

[Gm, Pm, Wcg, Wcp] = margin(oltf2);

Q16.wxo = Wcg;%(rad/s) Scalar

%% TABLE 5 

% table 5
WnRes = 0.1;
ZetaRes = 0.01;
TargPM = 40; % deg
OSu = 60 /100; % percent -> dec
Ts = 75 /100;
% Tr < final Ts (sec)
Ess = 0;

%% Q17
% p = -iir_wp;
% z1 = -Q16.wxo;
% z2 = -Q16.wxo;
% Dyn = -p / (z1 * z2) * (s - z1) * (s - z2) / s / (s - p);
% getMaxPM(Q16.K0*Q16.Dp*Q11.G*Q15.H, Q16.wxo, p, WnRes, ZetaRes)
% Q17.wn = -24.63147;
% Q17.zeta = -2.4;
% 
% Q17.Z = [Q17.zeta + 1i*sqrt(Q17.wn^2 - Q17.zeta^2), Q17.zeta - 1i*sqrt(Q17.wn^2 - Q17.zeta^2) ]; %(rad/s) 1x2 Vector
% 
% Q17.PM = 15.84; %(rad/s) Scalar
% Q17.D = (s^2 + 2*Q17.zeta*Q17.wn*s+ Q17.wn^2)/(Q17.wn^2); %(V/rad) LTI Object

Q17.Z = [-1.2453 + 2.5995i,  -1.2453 - 2.5995i];
Q17.PM = 102.9588;
Q17.D = Q16.Dp/(Q17.Z(1) * Q17.Z(2)) * (s-Q17.Z(1))*(s-(Q17.Z(2)));

% ======= MINE OWN ==========
% Q17.Z = [-1.351346  + 2.648226i,  -1.351346- 2.648226i];
% Q17.PM = 103.1313;
% Q17.D = Q16.Dp/(Q17.Z(1) * Q17.Z(2)) * (s-Q17.Z(1))*(s-(Q17.Z(2)));
% [num, den] = tfdata(Q16.Dp/(Q17.Z(1) * Q17.Z(2)) * (s-Q17.Z(1))*(s-(Q17.Z(2))), 'v'); % 'v' returns coefficients as vectors
% 
% num_real = real(num);
% den_real = real(den);
% 
% D17 = tf(num_real, den_real);
% Q17.D = D17;
%%
%Z, PM] = getMaxPM(Q16.K0*Q16.Dp*Q11.G*Q15.H, -Q16.wxo);

%%

search_max = 1; % tells us if we directly went to all the else statements (PM is fully maximized, changing zeta and wn doesnt increase PM anymore)
search = 1; % if search = 1, keep searching
WnRes = 0.1;
ZetaRes = 10^(-2);

% initial values
z1_max = 0; z2_max = 0;
wn_max = round(Q16.wxo,1);
zeta_max = 1;
r = roots([1 2*zeta_max*wn_max wn_max^2]);
z1 = r(1); z2 = r(2);
Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
[GM,PM_max] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);

% newtonian search
while search == 1;
    wn_current = wn_max;
    zeta_current = zeta_max;
    search = 0;

    % right point (increase wn)
    wn = wn_current + WnRes;
    zeta = zeta_current;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % left point (decrease wn)
    wn = wn_current - WnRes;
    zeta = zeta_current;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % top point (increase zeta)
    wn = wn_current;
    zeta = zeta_current + ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % bottom point (decrease zeta)
    wn = wn_current;
    zeta = zeta_current - ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end
    
    % top left point (increase zeta, decrease wn)
    wn = wn_current - WnRes;
    zeta = zeta_current + ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % top right point (increase zeta, increase wn)
    wn = wn_current + WnRes;
    zeta = zeta_current + ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % bottom right point (decrease zeta, increase wn)
    wn = wn_current + WnRes;
    zeta = zeta_current - ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

    % bottom left point (decrease zeta, decrease wn)
    wn = wn_current - WnRes;
    zeta = zeta_current - ZetaRes;
    r = roots([1 2*zeta*wn wn^2]);
    z1 = r(1); z2 = r(2);
    Q17.D = (s-z1)*(s-z2)/(z1*z2)*Q16.Dp;
    [GM,PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);
    if PM > PM_max && search == 0
        PM_max = PM;
        z1_max = z1; z2_max = z2;
        wn_max = wn; zeta_max = zeta;
        search = 1;
    end

end

% final values
Q17.Z = [z1_max,z2_max];
Q17.D = (s-Q17.Z(1))*(s-Q17.Z(2))/(Q17.Z(1)*Q17.Z(2))*Q16.Dp;
[GM,Q17.PM] = margin(Q16.K0*Q17.D*Q11.G*Q15.H);

%% 18
TargPM = 40;

Q18.K = Q16.K0*5.39;

%% 19
[num, den] = tfdata(Q16.Dp/(Q17.Z(1) * Q17.Z(2)) * (s-Q17.Z(1))*(s-(Q17.Z(2))), 'v'); % 'v' returns coefficients as vectors

num_real = real(num);
den_real = real(den);

D17 = tf(num_real, den_real);


z1 = Q17.Z(1);
z2 = Q17.Z(2);
p = -39.0673*1.1;
Q19.Kp = (1/p - (z1+z2)/(z1*z2)) * Q18.K;%(pure) Scalar
Q19.Kd = (1/(z1*z2) + Q19.Kp/p)*Q18.K*0.98;
Q19.Ki = Q18.K;%(sec‐1) Scalar


%% Q20
top = Q18.K*Q17.D*Q11.G; 
bot = Q15.H;
cltf = top/(1+top*bot);
fv = dcgain(cltf);
pv = 1.3758;
Q20.Tr = 0.1;%(sec) Scalar
Q20.Tp = 0.1773;%(sec) Scalar
Q20.Ts = 0.9645;%(sec) Scalar
Q20.OSu = 37.5833;%(%) Scalar
Q20.OSy = (pv/dcgain(cltf) - 1)*100;%(%) Scalar
Q20.Ess = abs(1-dcgain(cltf))*100;

%% Q21

% target vals
Ts = 0.75 * Q20.Ts;
Osu = 0.6 * Q20.OSu;
p = -39.0673*1.1;

%%
Kin.K = 1;
Kin.Kd = Q19.Kd;
Kin.Ki = Q19.Ki;
Kin.Kp = Q19.Kp;
heurRCGTune(Kin,Ts,Osu,0, p,Q11.G,Q15.H);




Q21.K = 1;
Q21.Kp = 0.1306;
Q21.Ki = 0.6462;
Q21.Kd = 0.0533;
%%
% Q21.K = 0;
% Q21.Kp = 0;
% Q21.Ki = 0;
% Q21.Kd = 0;
% 
% p = -39.0673*1.1;
% fwd = Q11.G*Q17.D;
% fdb = Q15.H;
% GAIN.O.K = Q18.K;
% GAIN.O.Kp = Q19.Kp;
% GAIN.O.Ki = Q19.Ki;
% GAIN.O.Kd = Q19.Kd;
% GAIN.N.K = Q21.K;
% GAIN.N.Kp = Q21.Kp;
% GAIN.N.Ki = Q21.Ki;
% GAIN.N.Kd = Q21.Kd;
% 
% [cltf, oltf] = heurTune('KPID',1000,fwd,fdb,GAIN,p);
%%
p = -39.0673*1.1;
Q22.D = Q21.Kp + Q21.Ki/s + Q21.Kd * -p*s/(s-p);
cltf = Q11.G * Q22.D / (1 + Q11.G * Q22.D * Q15.H);
stepinfo(cltf)


%% Q22

Q22.Tr = 0.14;
Q22.Tp = 0.2289;
Q22.Ts = 0.7006;
Q22.OSu = 20.9949*1.06;
Q22.OSy = 20.9949;
Q22.Ess = 0;















