%% FC2_all_cases.m
%  F-104A lateral-directional dynamics, FLIGHT CONDITION 2 (sea level, Mach 0.80)
%  Runs all three input cases in one Simulink model:
%     case 1 : free response, beta(0) = 2 deg, other states zero
%     case 2 : aileron pulse, 2 deg for 1 s
%     case 3 : rudder pulse,  2 deg for 1 s
%
%  Needs in the same folder:  F104_build_model.m
%  Produces: FC2_time_histories.png, FC2_results.mat, console summary and a PASS/CHECK
%            comparison with reference values (computed independently in Python).
%  Units: imperial (ft, slug, s). Model works in rad; plots are in deg.
%
%  NOTE: the model built here is called F104_FC2_sim, so it does not clash with
%        a model you built by hand called F104_FC2.

clear; clc; close all;

rebuild_model = true;     % true: rebuild plain model (erases 3D blocks); false: keep existing model
mdl = 'F104_FC2_sim';

%% ---------------- Flight-condition data (FC2) ----------------
g   = 32.2;   a0 = 1116.44;   rho = 0.00238;
Mach = 0.800;           the_d = 2.00;                     % Mach, trim attitude (deg)
yv  = -0.452;  lb = -146.0;  nb = 13.60;
lp  = -4.64;   np = -0.188;  lr = 3.67;   nr = -0.498;
yxi = 0;       lxi = 49.6;   nxi = 3.510;                 % aileron derivatives
yze = 0.0719;  lze = 41.50;  nze = -7.070;               % rudder derivatives

thetae = deg2rad(the_d);
Ve     = Mach*a0;                       % ft/s
qbar   = 0.5*rho*Ve^2;                  % lbf/ft^2

%% ---------------- State-space model:  x = [beta p r phi]',  u = [aileron rudder]' ----------------
A = [ yv,  sin(thetae), -cos(thetae),  g*cos(thetae)/Ve;
      lb,  lp,           lr,            0;
      nb,  np,           nr,            0;
      0,   1,            tan(thetae),   0 ];
B = [ yxi, yze;
      lxi, lze;
      nxi, nze;
      0,   0 ];
C = eye(4);   D = zeros(4,2);

fprintf('FC2: Mach %.3f, Ve = %.2f ft/s, q = %.2f lbf/ft^2\n', Mach, Ve, qbar);
disp('A ='); disp(A);   disp('B ='); disp(B);

%% ---------------- Eigenvalues and mode parameters ----------------
%  Expected: Dutch roll -0.3826 +/- 4.4874j (zeta 0.085, wn 4.50 rad/s, period 1.40 s)
%            roll -4.818 1/s (tau 0.208 s),  spiral -0.00724 1/s (time to half about 96 s)
ev = eig(A);
[dr, roll, spiral] = classify_modes(ev);
zeta_d = -real(dr)/abs(dr);   wn_d = abs(dr);   Td = 2*pi/imag(dr);
fprintf('\nDutch roll : %.4f %+.4fj   zeta = %.4f   wn = %.3f rad/s   period = %.2f s\n', ...
        real(dr), imag(dr), zeta_d, wn_d, Td);
fprintf('Roll mode  : %.4f 1/s   tau = %.3f s\n', roll, -1/roll);
if spiral > 0
    fprintf('Spiral     : %.5f 1/s   DIVERGENT, time to double = %.0f s\n', spiral, log(2)/spiral);
else
    fprintf('Spiral     : %.5f 1/s   convergent, time to half = %.0f s\n', spiral, log(2)/abs(spiral));
end
modes = struct('dutch_roll',dr,'zeta',zeta_d,'wn',wn_d,'period',Td, ...
               'roll_root',roll,'roll_tau',-1/roll,'spiral_root',spiral);

%% ---------------- Simulink model ----------------
if rebuild_model
    F104_build_model(mdl);
else
    load_system(mdl);
end

% variables used only by the optional 3D visualization blocks (harmless otherwise)
V_ms = Ve*0.3048;   alt_m = 300;   pos_scale = 0;          %#ok<NASGU>  pos_scale 0 = attitude only

caseNames = {'Free response, \beta(0) = 2 deg','Aileron pulse 2 deg, 1 s','Rudder pulse 2 deg, 1 s'};
ylab      = {'\beta (deg)','p (deg/s)','r (deg/s)','\phi (deg)'};
T_all = cell(1,3);   Y_all = cell(1,3);

fig = figure('Name','FC2 time histories','Position',[80 40 1150 820]);
tl  = tiledlayout(4,3,'TileSpacing','compact');

for c = 1:3
    % ---- set the case (workspace variables read by the model blocks) ----
    x0 = zeros(4,1);   ail_amp = 0;   rud_amp = 0;
    switch c
        case 1, x0(1)   = deg2rad(2);     % beta(0) = 2 deg
        case 2, ail_amp = deg2rad(2);     % 2 deg aileron pulse, 1 s
        case 3, rud_amp = deg2rad(2);     % 2 deg rudder pulse, 1 s
    end
    out = sim(mdl);
    T_all{c} = out.tout;   Y_all{c} = out.y_deg;      % y = [beta p r phi] (deg, deg/s)

    for s = 1:4
        nexttile((s-1)*3 + c);
        plot(T_all{c}, Y_all{c}(:,s), 'LineWidth',1.3);  grid on
        if c == 1, ylabel(ylab{s}); end
        if s == 1, title(caseNames{c}); end
        if s == 4, xlabel('t (s)'); end
    end
end
title(tl, sprintf('F-104A FC2: Mach %.3f, q = %.0f lbf/ft^2', Mach, qbar));
exportgraphics(fig,'FC2_time_histories.png','Resolution',150);

%% ---------------- Checks against reference values ----------------
%  Reference [beta p r phi] in deg, deg/s at t = 1, 10, 20 s (independent Python calculation)
tq  = [1 10 20];
ref = { [ -0.32  26.25  -4.67    3.08;      % case 1
           0.03  -1.00   0.09   -0.37;
          -0.00  -0.01  -0.01   -0.33 ], ...
        [ -0.05  25.61   1.19   20.21;      % case 2
           0.03  -0.33   0.82   23.07;
           0.02  -0.18   0.76   21.44 ], ...
        [  1.01 -21.42   1.21   -8.71;      % case 3
          -0.05   0.74  -0.30   -9.18;
          -0.01   0.09  -0.30   -8.41 ] };
fprintf('\n===== Check against reference (t = 1, 10, 20 s) =====\n');
for c = 1:3
    [tu, iu] = unique(T_all{c});
    ysim = interp1(tu, Y_all{c}(iu,:), tq);
    err  = abs(ysim - ref{c});
    tol  = 0.1 + 0.01*abs(ref{c});                 % 0.1 + 1 % of the reference value
    if all(err(:) <= tol(:)), verdict = 'PASS'; else, verdict = 'CHECK'; end
    fprintf('\nCase %d (%s): %s, max error = %.3f\n', c, regexprep(caseNames{c},'\\',''), verdict, max(err(:)));
    fprintf('   t(s)    beta      p        r       phi\n');
    for i = 1:3
        fprintf('  %4.0f  %8.2f %8.2f %8.2f %8.2f   (sim)\n', tq(i), ysim(i,:));
        fprintf('        %8.2f %8.2f %8.2f %8.2f   (ref)\n',        ref{c}(i,:));
    end
end

fprintf('\nPeak |value| over 0-20 s:\n   case    beta      p        r       phi\n');
for c = 1:3
    fprintf('    %d   %8.2f %8.2f %8.2f %8.2f\n', c, max(abs(Y_all{c}),[],1));
end

%% ---------------- Save results for the cross-flight-condition comparison ----------------
save('FC2_results.mat','modes','T_all','Y_all','A','B','Mach','qbar','Ve');
fprintf('\nSaved FC2_results.mat and FC2_time_histories.png\n');

%% ---------------- Local function ----------------
function [dr, roll, spiral] = classify_modes(ev)
% Dutch roll = complex pair; roll = most negative real root; spiral = other real root
    cplx   = ev(imag(ev) > 1e-9);
    dr     = cplx(1);
    re     = sort(real(ev(abs(imag(ev)) <= 1e-9)));
    roll   = re(1);
    spiral = re(2);
end