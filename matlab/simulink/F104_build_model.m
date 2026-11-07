function F104_build_model(mdl)
% F104_BUILD_MODEL  Builds a Simulink model (default name 'F104_lateral'; one model, all cases).
% Usage:  F104_build_model            -> model 'F104_lateral'
%         F104_build_model('NAME')    -> model 'NAME'  (used by the per-flight-condition scripts)
%
% Signal flow:
%   Pulse da --\
%               Mux -> State-Space (A,B,C,D,x0) -> Gain (180/pi) -> Demux -> 4 Scopes
%   Pulse dr --/                                         |
%                                                         +-> To Workspace (y_deg)
%
% The block parameters are workspace VARIABLES, not numbers, so the same
% model is reused for every flight condition and every input case:
%   A, B, C, D      state-space matrices      (x = [beta p r phi]', u = [da dr]')
%   x0              initial state, rad
%   ail_amp         aileron pulse amplitude, rad
%   rud_amp         rudder pulse amplitude, rad
% Run these in the BASE workspace before calling sim('F104_lateral').

if nargin < 1, mdl = 'F104_lateral'; end
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% ---- Inputs: single 1 s pulses (period 100 s, width 1 %, same style as the lab manual) ----
add_block('simulink/Sources/Pulse Generator', [mdl '/Pulse da'], ...
    'PulseType','Time based','Amplitude','ail_amp','Period','100', ...
    'PulseWidth','1','PhaseDelay','0','Position',[30 110 110 140]);
add_block('simulink/Sources/Pulse Generator', [mdl '/Pulse dr'], ...
    'PulseType','Time based','Amplitude','rud_amp','Period','100', ...
    'PulseWidth','1','PhaseDelay','0','Position',[30 170 110 200]);

% ---- Mux: input 1 = aileron, input 2 = rudder (order matches the columns of B) ----
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], ...
    'Inputs','2','Position',[150 110 155 200]);

% ---- Aircraft: lateral-directional state-space ----
add_block('simulink/Continuous/State-Space', [mdl '/Lateral SS'], ...
    'A','A','B','B','C','C','D','D','X0','x0','Position',[200 120 300 190]);

% ---- rad -> deg for plotting ----
add_block('simulink/Math Operations/Gain', [mdl '/rad2deg'], ...
    'Gain','180/pi','Position',[340 140 390 170]);

% ---- Demux and scopes ----
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Outputs','4','Position',[430 80 435 260]);

names = {'Scope beta','Scope p','Scope r','Scope phi'};
for i = 1:4
    y0 = 70 + (i-1)*55;
    add_block('simulink/Sinks/Scope', [mdl '/' names{i}], ...
        'NumInputPorts','1','Position',[500 y0 540 y0+30]);
end

% ---- Logging block (used by F104_lateral_main for the plots) ----
add_block('simulink/Sinks/To Workspace', [mdl '/Log y'], ...
    'VariableName','y_deg','SaveFormat','Array','MaxDataPoints','inf', ...
    'Position',[340 290 420 320]);

% ---- Wiring ----
add_line(mdl,'Pulse da/1','Mux/1','autorouting','on');
add_line(mdl,'Pulse dr/1','Mux/2','autorouting','on');
add_line(mdl,'Mux/1','Lateral SS/1','autorouting','on');
add_line(mdl,'Lateral SS/1','rad2deg/1','autorouting','on');
add_line(mdl,'rad2deg/1','Demux/1','autorouting','on');
add_line(mdl,'rad2deg/1','Log y/1','autorouting','on');
for i = 1:4
    add_line(mdl,sprintf('Demux/%d',i),[names{i} '/1'],'autorouting','on');
end

% ---- Solver settings (as in the lab manual) ----
set_param(mdl,'Solver','ode45','MaxStep','0.01','StopTime','20', ...
    'RelTol','1e-6','AbsTol','1e-8', ...
    'SaveTime','on','TimeSaveName','tout','ReturnWorkspaceOutputs','on');

save_system(mdl);
fprintf('Model %s built and saved.\n', mdl);
end