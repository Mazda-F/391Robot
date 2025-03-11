clear all; clc;
SN = 48082044;

%% PARAMS
A = 14; B = 18; C = 10; D = 18; E = 12; F = 10; G = 14; H = 14;

s = tf('s');

dhs = A*1e-2;
dls = B*4*1e-2;
npw = C/10;
jhs = D*3*1e-3;
jls = E*5*1e-2;
jp = F*5*1e-1;
mb = G*40;
bhs = H*3*1e-3;
bls = A*3*1e-3;
bbb = B*1e-3;
bpw = C*1e-1;
bbw = D*20;

g1 = G*100;
g2 = H*200;

rw = A*1e-2;
lw = B*5*1e-2;
km = C*2*1e-2;
jm = D*1e-2;
bm = E*1e-4;

CF = F*50;
DC = 0.7;

%% q1
tp = 2.16346/1000;
pv = 18.7562;
fv = 17.525;
tr = 1.74/1000;
os = pv/fv-1;
zeta = -log(os)/sqrt(pi^2+log(os)^2);
beta = sqrt(1-zeta^2);
wn = 1/tr/beta*(pi-atan(beta/zeta));
Q1.Ga = fv*wn^2/(s^2+2*zeta*wn*s+wn^2);

%% q2
G1 = g1/(s+g1);
G2 = g2/(s+g2);
hstemp = G1*G2/(1+G1*G2+G1*G1*G2*G2);
Q2.Hs = 1/s * hstemp;
Q2.Ks = dcgain(hstemp);
Q2.Ds = Q2.Hs/Q2.Ks;

%% q3
n1 = 1/(dhs/dls);
n2 = npw*2*pi;


Q3.Mj = (jm+jhs)*n1^2 *n2^2 + (jls+jp)*n2^2 +mb;
Q3.Bj = (bm+bhs) * n1^2 * n2^2 + (bbb+bls+bpw)*n2^2+bbw;
Q3.Kj = 0;
Q3.Jj = Q3.Mj;
%% q4
nkm = km *n1*n2;
Q4.Ye = 1/(s*lw+rw);
Q4.Ym = 1/(s*Q3.Jj+Q3.Bj+Q3.Kj/s);

top = nkm*Q4.Ye*Q4.Ym;
Q4.Gp = top/(1+nkm*top);

%% q5
 Q5.A = [
     -rw/lw -nkm/lw;
     nkm/Q3.Mj -Q3.Bj/Q3.Mj
 ];
 Q5.B = [
     1/lw; 0
     ];
 Q5.C = [
     jp/Q3.Jj/n2^2*nkm n2*(bbb+bpw);
     0 n1*n2*60/2/pi
     ];
 Q5.D = [0;0];

 %% q6
Q6.GHs = Q1.Ga*Q4.Gp*Q2.Hs;

 Q6.wd = 0.2957;
 ndp = 3.0;
 Q6.Nf = CF/ndp;
Q6.beta = Q6.Nf/(Q6.Nf+1);
dt = 1/CF;
Q6.tau = -dt/log(Q6.beta);

%% q7
[~, ~, Q7.num] = getFIR(Q6.beta);

%% q8

Q8.N = Q6.Nf + 0.5+DC+0.5;
Q8.N = Q8.N;
wc = CF/Q8.N;
Q8.Hc = s*wc/(s+wc)*1/Q2.Ks;

%% q9
Q9.G = Q1.Ga*Q4.Gp;
 Q9.H = Q8.Hc*Q2.Hs;
 Q9.GH =  Q9.G*Q9.H;
 Q9.Ktj = 1;
 Q9.Kjt = 1;

 %% q10
 GH = Q9.GH;
wp = wc;
Q10.Dp = wp/(s+wp)/s; %(pure) LTI Object

oltf = Q10.Dp * GH;
[gm, ~,~,~] = margin(oltf);

Q10.K0 = gm;
oltf = Q10.K0*Q10.Dp * GH;
[~,~,wcg,~] = margin(oltf);
Q10.wxo = wcg;

%% q11
Q11.Z = [-0.1860+0.2354i , -0.1860-0.2354i];
Q11.PM = 87.856338;
Q11.D = (2.97*s^2 + 1.105*s + 0.2673)/(0.09*s^2 + 0.2673*s);

%% q12
targpm = 60;
Q12.K = Q10.K0*2.45;
% [~,pm,~,~] = margin(Q12.K*Q11.D*GH)
%% q13
p = -wp;
z1 = Q11.Z(1);
z2 = Q11.Z(2);
zsum = real(z1)*2;
z1z2 = real(z1)^2 + imag(z1)^2;
Q13.Kp = (1/p - (zsum)/(z1z2)) * Q12.K;
Q13.Ki = Q12.K;
Q13.Kd = (1/p^2 - (zsum-p)/(p*z1z2)) * Q12.K;

%% q14
top = Q12.K*Q11.D*Q9.G;
bot = Q9.H;
cltf = top/(1+top*bot);
fv = dcgain(cltf);
pv = 1.0930;
Q14.Tr = 1.44; 
Q14.Tp = 1.9699;
Q14.Ts = 12.4220;
Q14.OSu = (pv-1)*100;
Q14.OSy = 9.3043;
Q14.Ess = abs(1-fv)*100;

%% q15
% Ktune(Q13,Q9.G,Q9.H,p, 0.8*Q14.Ts, 0.5*Q14.OSu)

Q15.Kp = 0.86*Q13.Kp;
Q15.Ki = 0.75*Q13.Ki;
Q15.Kd = 0.6*Q13.Kd;

%% q16
Dnew = (Q15.Kp) + (Q15.Ki/s) + (Q15.Kd * -p * s / (s - p));
top = Dnew*Q9.G;
bot = Q9.H;
cltf = top/(1+top*bot);
fv = dcgain(cltf);
pv = 1.0262;
Q16.Tr = 2.41; 
Q16.Tp = 3.0126;
Q16.Ts = 3.4390;
Q16.OSu = (pv-1)*100*0.98;
Q16.OSy = 2.6168*0.98;
Q16.Ess = abs(1-fv)*100;