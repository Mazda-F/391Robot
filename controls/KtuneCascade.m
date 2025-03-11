%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ktune()
% GUI for fast PID Tuning.
% IMPORTANT: To adjust slider range, edit the code below
%
% Syntax:
%   Ktune(K, G, H, p, Ts, OSu)
%
% Example Usage:
%   Ktune(Q13, Q9.G, Q9.H, p,  0.7*Q14.Ts,  0.4*Q14.OSu)
% 
% Input:
%   K   (struct)    Struct containing old Kp, Ki, Kd to be tuned. 
%   G               Forward path gain
%   H               Feedback path gain
%   p               Derivative pole
%   Ts              Rise time target
%   OSu             Overshoot target
%
% GUI elements:
%   Blue line       - Always stays at 1
%   Yellow lines    - Represents +/- 2% error margin from 1.
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Ktune2(K, X, T, Ts, OSu)
    
    % main gui config
    fig = uifigure('Name', 'Ktune2', 'Position', [100, 100, 1600, 900]);
    ax = uiaxes(fig, 'Position', [50, 300, 1200, 800]);
    ax.Title.String = 'CLTF Step Response (X & T)';
    ax.XLabel.String = 'Time (s)';
    ax.YLabel.String = 'Response';
    
    % p_defval = 0.1;
    % d_defval = 0.01;
    % K.Kpx = p_defval;
    % K.Kix = p_defval;
    % K.Kdx = d_defval;
    % 
    % K.Kpt = p_defval;
    % K.Kit = p_defval;
    % K.Kdt = d_defval;

    maxval = 3.2;
        
    % NOTE: IF THE SLIDER RANGE ISNT ENOUGH,
    % CHANGE [0,2] ------------------------------------------------------
    %                                                                   |
    % TO YOUR DESIRED RANGE                                             \/
    %
    % X KP SLIDER CONFIG
    XsliderKp = uislider(fig, 'Position', [100, 260, 600, 4], 'Limits', [0, maxval], 'Value', 1);
    XlabelKp = uilabel(fig, 'Position', [100, 280, 300, 20], 'Text', 'X Kp Multiplier: 1');
    
    % X KI SLIDER CONFIG
    XsliderKi = uislider(fig, 'Position', [100, 160, 600, 4], 'Limits', [0, maxval], 'Value', 1);
    XlabelKi = uilabel(fig, 'Position', [100, 180, 200, 20], 'Text', 'X Ki Multiplier: 1');
    
    % X KD SLIDER CONFIG
    XsliderKd = uislider(fig, 'Position', [100, 60, 600, 4], 'Limits', [0, maxval], 'Value', 1);
    XlabelKd = uilabel(fig, 'Position', [100, 80, 200, 20], 'Text', 'X Kd Multiplier: 1');

    % T KP SLIDER CONFIG
    TsliderKp = uislider(fig, 'Position', [800, 260, 600, 4], 'Limits', [0, maxval], 'Value', 1);
    TlabelKp = uilabel(fig, 'Position', [800, 280, 300, 20], 'Text', 'θ Kp Multiplier: 1');
    
    % T KI SLIDER CONFIG
    TsliderKi = uislider(fig, 'Position', [800, 160, 600, 5], 'Limits', [0, maxval], 'Value', 1);
    TlabelKi = uilabel(fig, 'Position', [800, 180, 200, 20], 'Text', 'θ Ki Multiplier: 1');
    
    % T KD SLIDER CONFIG
    TsliderKd = uislider(fig, 'Position', [800, 60, 600, 4], 'Limits', [0, maxval], 'Value', 1);
    TlabelKd = uilabel(fig, 'Position', [800, 80, 200, 20], 'Text', 'θ Kd Multiplier: 1');


    TresKp = uilabel(fig, 'Position', [1500, 580, 200, 20], 'Text', 'θ Kp: 1', 'FontWeight', 'Bold');
    TresKi = uilabel(fig, 'Position', [1500, 480, 200, 20], 'Text', 'θ Ki: 1', 'FontWeight', 'Bold');
    TresKd = uilabel(fig, 'Position', [1500, 380, 200, 20], 'Text', 'θ Kd: 1', 'FontWeight', 'Bold');
    XresKp = uilabel(fig, 'Position', [1500, 280, 200, 20], 'Text', 'X Kp: 1', 'FontWeight', 'Bold');
    XresKi = uilabel(fig, 'Position', [1500, 180, 200, 20], 'Text', 'X Ki: 1', 'FontWeight', 'Bold');
    XresKd = uilabel(fig, 'Position', [1500, 80, 200, 20], 'Text', 'X Kd: 1', 'FontWeight', 'Bold');

    % init plot
    updatePlot();

    % slider call back funcs
    XsliderKp.ValueChangedFcn = @(src, event) updatePlot();
    XsliderKi.ValueChangedFcn = @(src, event) updatePlot();
    XsliderKd.ValueChangedFcn = @(src, event) updatePlot();
    TsliderKp.ValueChangedFcn = @(src, event) updatePlot();
    TsliderKi.ValueChangedFcn = @(src, event) updatePlot();
    TsliderKd.ValueChangedFcn = @(src, event) updatePlot();

    % plotting func
    function updatePlot()
        
        XKpMultiplier = XsliderKp.Value;
        XKiMultiplier = XsliderKi.Value;
        XKdMultiplier = XsliderKd.Value;
        
        TKpMultiplier = TsliderKp.Value;
        TKiMultiplier = TsliderKi.Value;
        TKdMultiplier = TsliderKd.Value;


        XlabelKp.Text = sprintf('X Kp Multiplier: %.4f', XKpMultiplier);
        XlabelKi.Text = sprintf('X Ki Multiplier: %.4f', XKiMultiplier);
        XlabelKd.Text = sprintf('X Kd Multiplier: %.4f', XKdMultiplier);

        TlabelKp.Text = sprintf('θ Kp Multiplier: %.4f', TKpMultiplier);
        TlabelKi.Text = sprintf('θ Ki Multiplier: %.4f', TKiMultiplier);
        TlabelKd.Text = sprintf('θ Kd Multiplier: %.4f', TKdMultiplier);

        XresKp.Text = sprintf('X Kp: %.4f', XKpMultiplier*K.Kpx);
        XresKi.Text = sprintf('X Ki: %.4f', XKiMultiplier*K.Kix);
        XresKd.Text = sprintf('X Kd: %.4f', XKdMultiplier*K.Kdx);
        TresKp.Text = sprintf('θ Kp: %.4f', TKpMultiplier*K.Kpt);
        TresKi.Text = sprintf('θ Ki: %.4f', TKiMultiplier*K.Kit);
        TresKd.Text = sprintf('θ Kd: %.4f', TKdMultiplier*K.Kdt);


        s = tf('s');

        % Dx = (K.Kpx * XKpMultiplier) + (K.Kix * XKiMultiplier / s) + (K.Kdx * XKdMultiplier * -X.p * s / (s - X.p));
        % 
        % Dt = (K.Kpt * TKpMultiplier) + (K.Kit * TKiMultiplier / s) + (K.Kdt * TKdMultiplier * -T.p * s / (s - T.p));

        Dx = (XKpMultiplier) + (XKiMultiplier / s) + (XKdMultiplier * -X.p * s / (s - X.p));

        Dt = (TKpMultiplier) + (TKiMultiplier / s) + (TKdMultiplier * -T.p * s / (s - T.p));
        
        
        cltf_t = Dt*T.G/(1+Dt*T.GH);
        cltf_x = cltf_t * Dx / (1 + cltf_t * X.H * Dx);


        [y, t] = step(cltf_x);

        % plot
        cla(ax); % clear axes
        plot(ax, t, y, 'k-', 'LineWidth', 1.5);
        hold(ax, 'on');
        plot(ax, xlim(ax), [1, 1], 'Color', [0.3010 0.7450 0.9330], 'LineStyle', '-', 'LineWidth', 1.1);
        plot(ax, xlim(ax), [1.02, 1.02], 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '-', 'LineWidth', 1.1);
        plot(ax, xlim(ax), [0.98, 0.98], 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '-', 'LineWidth', 1.1);
        plot(ax, xlim(ax), [(OSu / 100 + 1), (OSu / 100 + 1)], 'r--', 'LineWidth', 1.5);
        plot(ax, [Ts, Ts], ylim(ax), 'r--', 'LineWidth', 1.5);
        hold(ax, 'off');

        % auto axis adjust
        % xLimits = [min(t), max(t)];
        % yLimits = [min(y), max(y)];
        xLimits = [0, 1];
        yLimits = [0, abs(max(y))];
        ylim(ax, [yLimits(1) - 0.05 * abs(yLimits(1)), yLimits(2) + 0.05 * abs(yLimits(2))]);
        xlim(ax, xLimits);

        grid(ax, 'on');
    end
end
