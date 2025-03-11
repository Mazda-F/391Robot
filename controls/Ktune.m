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
function Ktune(K, G, H, p, Ts, OSu)
    
    % main gui config
    fig = uifigure('Name', 'Ktune', 'Position', [100, 100, 800, 900]);
    ax = uiaxes(fig, 'Position', [50, 300, 700, 600]);
    ax.Title.String = 'CLTF Step Response';
    ax.XLabel.String = 'Time (s)';
    ax.YLabel.String = 'Response';
        
    % NOTE: IF THE SLIDER RANGE ISNT ENOUGH,
    % CHANGE [0,2] ------------------------------------------------------
    %                                                                   |
    % TO YOUR DESIRED RANGE                                             \/
    %
    % KP SLIDER CONFIG
    sliderKp = uislider(fig, 'Position', [100, 260, 600, 4], 'Limits', [0, 15], 'Value', 1);
    labelKp = uilabel(fig, 'Position', [100, 280, 300, 20], 'Text', 'Kp Multiplier: 1');
    
    % KI SLIDER CONFIG
    sliderKi = uislider(fig, 'Position', [100, 160, 600, 4], 'Limits', [0, 15], 'Value', 1);
    labelKi = uilabel(fig, 'Position', [100, 180, 200, 20], 'Text', 'Ki Multiplier: 1');
    
    % KD SLIDER CONFIG
    sliderKd = uislider(fig, 'Position', [100, 60, 600, 4], 'Limits', [0, 15], 'Value', 1);
    labelKd = uilabel(fig, 'Position', [100, 80, 200, 20], 'Text', 'Kd Multiplier: 1');

    % init plot
    updatePlot();

    % slider call back funcs
    sliderKp.ValueChangedFcn = @(src, event) updatePlot();
    sliderKi.ValueChangedFcn = @(src, event) updatePlot();
    sliderKd.ValueChangedFcn = @(src, event) updatePlot();

    % plotting func
    function updatePlot()
        
        KpMultiplier = sliderKp.Value;
        KiMultiplier = sliderKi.Value;
        KdMultiplier = sliderKd.Value;

        labelKp.Text = sprintf('Kp Multiplier: %.4f', KpMultiplier);
        labelKi.Text = sprintf('Ki Multiplier: %.4f', KiMultiplier);
        labelKd.Text = sprintf('Kd Multiplier: %.4f', KdMultiplier);

        s = tf('s');
        D = (K.Kp * KpMultiplier) + (K.Ki * KiMultiplier / s) + (K.Kd * KdMultiplier * -p * s / (s - p));
        cltf = G * D / (1 + G * H * D);
        
        [y, t] = step(cltf);

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
