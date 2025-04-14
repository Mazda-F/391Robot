%% Control Blocks %%
%% Ye - Motor Windings 
function Ye = importYe()
% Determined Experimentally.
s = tf('s');
Rw = 7.4; % from ohmeter
Rtest = 10.47;
Rtot = Rw+Rtest;

showdata = 0;
if showdata
    Lw_data = readmatrix('TEK00001.CSV', Range='24:2011');
    Lw_time = Lw_data(:,1);
    Lw_voltage = Lw_data(:,2);
    plot(Lw_time, Lw_voltage, Color='k')
end

tau = 1.6e-7 - 8e-9;
Lw = Rtot * tau;
% fprintf("%.12f", Lw)
Ye = 1/(s*Lw + Rw);
end