%% F104_mode_comparison.m  (formerly F104_lateral_main.m / F104_FC1_M_vs_q.m)
%  Team 15 - F-104A lateral-directional dynamics at three sea-level flight conditions
%
%  Deliverables produced by this script
%   1. State-space model (A,B,C,D) for FC1, FC2, FC3
%   2. Eigenvalues, damping ratios, natural frequencies, time constants
%   3. Mode comparison charts vs Mach number and dynamic pressure
%   4. Time histories of beta, p, r, phi (free response, aileron pulse, rudder pulse)
%   5. Console summary + CSV used for the written interpretation
%
%  Requires: MATLAB + Simulink. Put F104_build_model.m in the same folder.
%  Units: imperial (ft, slug, s). Angles in rad inside the model, deg on plots.

clear; clc; close all;

%% ---------------- Constants and flight-condition data ----------------
g   = 32.2;          % ft/s^2
a0  = 1116.44;       % ft/s
rho = 0.00238;       % slug/ft^3

%            FC1      FC2      FC3
Mach   = [  0.257    0.800    1.100 ];
the_d  = [  2.30     2.00     1.00  ];            % trim attitude, deg
yv  = [ -0.178   -0.452   -0.791 ];
lb  = [ -20.9    -146.0   -363.0 ];
nb  = [  2.68     13.60    42.70 ];
lp  = [ -1.38    -4.64    -7.12  ];
np  = [ -0.0993  -0.188   -0.341 ];
lr  = [  1.16     3.67     7.17  ];
nr  = [ -0.157   -0.498   -1.060 ];
yxi = [  0        0        0     ];
lxi = [  4.76     49.6     81.5  ];
nxi = [  0.266    3.510    6.500 ];
yze = [  0.0317   0.0719   0.0621];
lze = [  5.35     41.50    57.60 ];
nze = [ -0.923   -7.070   -8.720 ];

Ve = Mach*a0;                 % true airspeed, ft/s
q  = 0.5*rho*Ve.^2;           % dynamic pressure, lbf/ft^2

%% ---------------- 1. State-space models ----------------
%  x = [beta p r phi]',   u = [aileron rudder]'
A_all = cell(1,3);  B_all = cell(1,3);
for k = 1:3
    th = deg2rad(the_d(k));
    A_all{k} = [ yv(k),  sin(th), -cos(th),  g*cos(th)/Ve(k);
                 lb(k),  lp(k),    lr(k),     0;
                 nb(k),  np(k),    nr(k),     0;
                 0,      1,        tan(th),   0 ];
    B_all{k} = [ yxi(k), yze(k);
                 lxi(k), lze(k);
                 nxi(k), nze(k);
                 0,      0 ];
end
C = eye(4);  D = zeros(4,2);

for k = 1:3
    fprintf('\n===== FC%d: Mach %.3f, Ve = %.1f ft/s, q = %.1f lbf/ft^2 =====\n', ...
            k, Mach(k), Ve(k), q(k));
    disp('A ='); disp(A_all{k});
    disp('B ='); disp(B_all{k});
    disp('C ='); disp(C);
    disp('D ='); disp(D);
end

%% ---------------- 2. Eigenvalues and mode parameters ----------------
res = zeros(3,12);
for k = 1:3
    ev = eig(A_all{k});
    [dr, roll, spiral] = classify_modes(ev);
    zeta_d = -real(dr)/abs(dr);
    wn_d   = abs(dr);
    Td     = 2*pi/imag(dr);
    tau_r  = -1/roll;
    t12_s  = log(2)/abs(spiral);        % time to half (stable) or double (unstable)
    res(k,:) = [k Mach(k) q(k) real(dr) imag(dr) zeta_d wn_d Td roll tau_r spiral t12_s];
end
T = array2table(res, 'VariableNames', {'FC','Mach','q_psf','DR_real','DR_imag', ...
    'DR_zeta','DR_wn','DR_period','Roll_root','Roll_tau','Spiral_root','Spiral_t_half_or_double'});
disp(' '); disp('Mode summary (imperial units, time in s):'); disp(T);
writetable(T, 'F104_mode_summary.csv');

for k = 1:3
    if res(k,11) > 0, sp = 'DIVERGENT (time to double)'; else, sp = 'convergent (time to half)'; end
    if res(k,6)  < 0, dd = 'UNSTABLE'; else, dd = 'stable'; end
    fprintf('FC%d: Dutch roll %s (zeta=%.3f, wn=%.2f rad/s); roll tau=%.3f s; spiral %s, %.0f s\n', ...
        k, dd, res(k,6), res(k,7), res(k,10), sp, res(k,12));
end

%% ---------------- 3. Mode comparison charts ----------------
fig = figure('Name','Mode comparison','Position',[60 60 1300 560]);
tl = tiledlayout(2,4,'TileSpacing','compact');
xs = {Mach, q};  xl = {'Mach number','q (lbf/ft^2)'};
for r = 1:2
    nexttile; plot(xs{r}, res(:,6), 'o-','LineWidth',1.5); yline(0,'k'); grid on
    xlabel(xl{r}); ylabel('Dutch roll \zeta');
    nexttile; plot(xs{r}, res(:,7), 'o-','LineWidth',1.5); grid on
    xlabel(xl{r}); ylabel('Dutch roll \omega_n (rad/s)');
    nexttile; plot(xs{r}, res(:,10),'o-','LineWidth',1.5); grid on
    xlabel(xl{r}); ylabel('Roll time constant (s)');
    nexttile; plot(xs{r}, res(:,11),'o-','LineWidth',1.5); yline(0,'k'); grid on
    xlabel(xl{r}); ylabel('Spiral root (1/s)');
end
title(tl,'F-104A lateral-directional modes vs Mach number and dynamic pressure');
exportgraphics(fig,'F104_mode_comparison.png','Resolution',150);

%% ---------------- 4. Time histories (Simulink, one model for all cases) ----------------
F104_build_model;                                  % creates and saves F104_lateral.slx

caseNames = {'Free response, \beta(0) = 2 deg','Aileron pulse 2 deg, 1 s','Rudder pulse 2 deg, 1 s'};
ylab = {'\beta (deg)','p (deg/s)','r (deg/s)','\phi (deg)'};
peaks = zeros(9,4);  row = 0;

for k = 1:3
    A = A_all{k};  B = B_all{k};                  %#ok<NASGU>  used by the model blocks
    fig = figure('Name',sprintf('FC%d time histories',k),'Position',[80 40 1150 820]);
    tl = tiledlayout(4,3,'TileSpacing','compact');
    for c = 1:3
        x0 = zeros(4,1);  ail_amp = 0;  rud_amp = 0;      %#ok<NASGU>
%**************************************************************************************************************************************************
        switch c
            case 1, x0(1)  = deg2rad(2);          % beta(0) = 2 deg, other states zero
            case 2,  ail_amp = deg2rad(2);         % 2 deg aileron pulse, 1 s
            case 3,  rud_amp = deg2rad(2);         % 2 deg rudder pulse, 1 s
        end
%**************************************************************************************************************************************************
        out = sim('F104_lateral');
        t = out.tout;  y = out.y_deg;              % y = [beta p r phi] in deg, deg/s
        row = row + 1;  peaks(row,:) = max(abs(y),[],1);
        for s = 1:4
            nexttile((s-1)*3 + c);
            plot(t, y(:,s), 'LineWidth',1.3); grid on
            if c == 1, ylabel(ylab{s}); end
            if s == 1, title(caseNames{c}); end 
            if s == 4, xlabel('t (s)'); end
        end
    end
    title(tl, sprintf('F-104A FC%d: Mach %.3f, q = %.0f lbf/ft^2', k, Mach(k), q(k)));
    exportgraphics(fig, sprintf('F104_FC%d_time_histories.png',k), 'Resolution',150);
end

%% ---------------- 5. Peak values (compare with the reference table) ----------------
rowNames = {};
for k = 1:3
    for c = 1:3
        rowNames{end+1} = sprintf('FC%d_case%d',k,c); %#ok<SAGROW>
    end
end
P = array2table(peaks, 'VariableNames',{'max_beta_deg','max_p_degps','max_r_degps','max_phi_deg'}, ...
                'RowNames',rowNames);
disp('Peak |value| over 0-20 s  (case 1 = free, 2 = aileron pulse, 3 = rudder pulse):'); disp(P);
writetable(P,'F104_peak_values.csv','WriteRowNames',true);

%% ---------------- Local function ----------------
function [dr, roll, spiral] = classify_modes(ev)
% Dutch roll : the complex pair
% Roll       : the most negative real root (fast)
% Spiral     : the remaining real root (very slow)
    cplx   = ev(imag(ev) > 1e-9);
    dr     = cplx(1);
    re     = sort(real(ev(abs(imag(ev)) <= 1e-9)));
    roll   = re(1);
    spiral = re(2);
end