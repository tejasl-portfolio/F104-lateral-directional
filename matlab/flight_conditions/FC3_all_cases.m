%% FC3_all_cases.m
%  F-104A lateral-directional dynamics, FLIGHT CONDITION 3 (sea level, Mach 1.10)
%  Runs all three input cases in one Simulink model:
%     case 1 : free response, beta(0) = 2 deg, other states zero
%     case 2 : aileron pulse, 2 deg for 1 s
%     case 3 : rudder pulse,  2 deg for 1 s
%
%  Needs in the same folder:  F104_build_model.m
%  Produces: FC3_time_histories.png, FC3_results.mat, console summary and a PASS/CHECK
%            comparison with reference values (computed independently in Python).
%  Units: imperial (ft, slug, s). Model works in rad; plots are in deg.
%
%  NOTE: the model built here is called F104_FC3_sim, so it does not clash with
%        a model you built by hand called F104_FC3.

clear; clc; close all;

rebuild_model = true;     % true: rebuild plain model (erases 3D blocks); false: keep existing model
mdl = 'F104_FC3_sim';

%% ---------------- Flight-condition data (FC3) ----------------
g   = 32.2;   a0 = 1116.44;   rho = 0.00238;
Mach = 1.100;           the_d = 1.00;                     % Mach, trim attitude (deg)
yv  = -0.791;  lb = -363.0;  nb = 42.70;
lp  = -7.12;   np = -0.341;  lr = 7.17;   nr = -1.060;
yxi = 0;       lxi = 81.5;   nxi = 6.500;                 % aileron derivatives
yze = 0.0621;  lze = 57.60;  nze = -8.720;                % rudder derivatives

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

fprintf('FC3: Mach %.3f, Ve = %.2f ft/s, q = %.2f lbf/ft^2\n', Mach, Ve, qbar);
disp('A ='); disp(A);   disp('B ='); disp(B);

%% ---------------- Eigenvalues and mode parameters ----------------
%  Expected: Dutch roll -0.5527 +/- 7.5139j (zeta 0.073, wn 7.53 rad/s, period 0.84 s)
%            roll -7.861 1/s (tau 0.127 s),  spiral -0.00418 1/s (time to half about 166 s)
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

fig = figure('Name','FC3 time histories','Position',[80 40 1150 820]);
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
title(tl, sprintf('F-104A FC3: Mach %.3f, q = %.0f lbf/ft^2', Mach, qbar));
exportgraphics(fig,'FC3_time_histories.png','Resolution',150);

%% ---------------- Checks against reference values ----------------
%  Reference [beta p r phi] in deg, deg/s at t = 1, 10, 20 s (independent Python calculation)
tq  = [1 10 20];
ref = { [  0.40 -35.42   6.72   -2.40;      % case 1
           0.01  -0.14  -0.03   -0.27;
           0.00   0.00  -0.01   -0.29 ], ...
        [ -0.06  24.79   1.65   23.29;      % case 2
           0.02  -0.14   0.66   25.62;
           0.01  -0.11   0.63   24.57 ], ...
        [  0.29   3.79  -1.57   -2.27;      % case 3
           0.00   0.10  -0.09   -3.07;
           0.00   0.01  -0.08   -2.95 ] };
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

%  Expected peaks |value| over 0-20 s:  case 1: 2.00, 53.1, 11.9, 8.62
%                                       case 2: 0.17, 29.9, 1.65, 26.9
%                                       case 3: 0.66, 13.3, 2.50, 4.61
fprintf('\nPeak |value| over 0-20 s:\n   case    beta      p        r       phi\n');
for c = 1:3
    fprintf('    %d   %8.2f %8.2f %8.2f %8.2f\n', c, max(abs(Y_all{c}),[],1));
end

%% ---------------- Save results for the cross-flight-condition comparison ----------------
save('FC3_results.mat','modes','T_all','Y_all','A','B','Mach','qbar','Ve');
fprintf('\nSaved FC3_results.mat and FC3_time_histories.png\n');

%% ---------------- Local function ----------------
function [dr, roll, spiral] = classify_modes(ev)
% Dutch roll = complex pair; roll = most negative real root; spiral = other real root
    cplx   = ev(imag(ev) > 1e-9);
    dr     = cplx(1);
    re     = sort(real(ev(abs(imag(ev)) <= 1e-9)));
    roll   = re(1);
    spiral = re(2);
end