%% Ga - MOTOR
% Read from Oscilloscope, using resistor of 963.6 ohms
% The motor driver was driven by 3.3 V, scale this down.
function Ga = importGa()
    rtest = 963.6;
    vin = 3.3;
    s = tf('s');
    runthis = 0; % readcsv
    if runthis
        Ga_data = readmatrix('TEK00000.CSV', Range='24:2011');
        roi = 999:1325;
        Ga_time = Ga_data(:,1);
        Ga_time = Ga_time(roi,:);
        Ga_time = Ga_time - min(Ga_time); % shift to start at t = 0
        Ga_voltage = Ga_data(:,2) - 0.08;
        Ga_voltage = Ga_voltage(roi,:) ./ vin;
    end
    
    tau = 8e-8;
    tr1 = (1.35e-7 - 1e-8);
    zeta = tau/(3.86*tau-1.83*tr1);
    wn = 2.1*zeta/tau;
    kdc = 10.56/vin;
    Ga = kdc*wn^2/(s^2 + 2*zeta*wn*s + wn^2);
    
    plotthis = 0; % plot to see
    if plotthis
        plot(Ga_time, Ga_voltage)
        hold on
        step(Ga)
    end
end