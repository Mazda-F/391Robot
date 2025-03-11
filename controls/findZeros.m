%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% findZeros()
% Newtonian zero search
%
% Syntax:
%   [Z, PM, D ] = findZeros(K0, wxo, GH, Dp)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Z, PM, D] = findZeros(K0, wxo, GH, Dp)

    s = tf('s');

    % search_max = 1; % tells us if we directly went to all the else statements (PM is fully maximized, changing zeta and wn doesnt increase PM anymore)
    search = 1; % if search = 1, keep searching
    WnRes = 0.1;
    ZetaRes = 10^(-2);

    % initial values
    z1_max = 0; z2_max = 0;
    wn_max = round(wxo,1);
    zeta_max = 1;
    r = roots([1 2*zeta_max*wn_max wn_max^2]);
    z1 = r(1); z2 = r(2);
    D = (s-z1)*(s-z2)/(z1*z2)*Dp;
    [~,PM_max] = margin(K0*D*GH);

    % newtonian search
    while search == 1
        wn_current = wn_max;
        zeta_current = zeta_max;
        search = 0;

        % right point (increase wn)
        wn = wn_current + WnRes;
        zeta = zeta_current;
        r = roots([1 2*zeta*wn wn^2]);
        z1 = r(1); z2 = r(2);
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM,~,~] = margin(K0*D*GH);
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
        D = (s-z1)*(s-z2)/(z1*z2)*Dp;
        [~,PM] = margin(K0*D*GH);
        if PM > PM_max && search == 0
            PM_max = PM;
            z1_max = z1; z2_max = z2;
            wn_max = wn; zeta_max = zeta;
            search = 1;
        end

    end

    % final values
    Z = [z1_max,z2_max];
    D = (s-Z(1))*(s-Z(2))/(Z(1)*Z(2))*Dp;
    [~,PM] = margin(K0*D*GH);


    fprintf('Found zeros: %f%+fj\n', real(Z(1)), imag(Z(1)));
    fprintf('Found zeros: %f%+fj\n', real(Z(2)), imag(Z(2)));
    fprintf('Found PM: %f \n',PM);
end