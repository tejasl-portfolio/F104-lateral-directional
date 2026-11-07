function F104_add_visualization(mdl)
% F104_ADD_VISUALIZATION  Adds the "visualization path" to a model (default F104_lateral).
% Usage:  F104_add_visualization              -> model 'F104_lateral'
%         F104_add_visualization('NAME')      -> model 'NAME', e.g. 'F104_FC1_sim'
%
% Run order:
%   1) F104_build_model          (creates F104_lateral)
%   2) set the workspace variables for a flight condition (see below)
%   3) F104_add_visualization    (this file)
%   4) add the three 3D blocks by hand (instructions printed at the end)
%
% Blocks added (all standard Simulink blocks):
%   Demux rad -> splits the State-Space output [beta p r phi] (rad)
%   Heading psi = integral( r / cos(thetae) )
%   Track chi   = psi + beta
%   North, East = integral( V_ms*pos_scale * cos/sin(chi) )
%   Down        = -alt_m
%   Mux T       = [North East Down]  (m)       -> Body_T of the Airliner Pack
%   Mux R       = [phi thetae psi]   (rad)     -> Body_R of the Airliner Pack
%   Scope psi   = heading in degrees (quick check, Step 1 of the build order)
%
% Workspace variables needed by the new blocks:
%   thetae     trim attitude, rad        e.g. thetae = deg2rad(2.30)
%   V_ms       true airspeed, m/s        e.g. V_ms = 0.257*1116.44*0.3048
%   alt_m      display altitude, m       e.g. alt_m = 300
%   pos_scale  0 = attitude only, 1 = fly the ground track

if nargin < 1, mdl = 'F104_lateral'; end
load_system(mdl);

%% ---- Tap the State-Space output (radians, before the rad2deg gain) ----
add_block('simulink/Signal Routing/Demux', [mdl '/Demux rad'], ...
    'Outputs','4','Position',[330 360 335 460]);
add_line(mdl,'Lateral SS/1','Demux rad/1','autorouting','on');

%% ---- Heading psi = integral( r / cos(thetae) ) ----
add_block('simulink/Math Operations/Gain', [mdl '/Gain 1 over cos thetae'], ...
    'Gain','1/cos(thetae)','Position',[380 410 440 440]);
add_block('simulink/Continuous/Integrator', [mdl '/Heading psi'], ...
    'InitialCondition','0','Position',[470 410 500 440]);
add_line(mdl,'Demux rad/3','Gain 1 over cos thetae/1','autorouting','on');
add_line(mdl,'Gain 1 over cos thetae/1','Heading psi/1','autorouting','on');

% quick check: heading in degrees on a scope
add_block('simulink/Math Operations/Gain', [mdl '/Gain psi deg'], ...
    'Gain','180/pi','Position',[530 460 580 490]);
add_block('simulink/Sinks/Scope', [mdl '/Scope psi'], ...
    'NumInputPorts','1','Position',[610 460 650 490]);
add_line(mdl,'Heading psi/1','Gain psi deg/1','autorouting','on');
add_line(mdl,'Gain psi deg/1','Scope psi/1','autorouting','on');

%% ---- Ground track chi = psi + beta ----
add_block('simulink/Math Operations/Sum', [mdl '/Track chi'], ...
    'Inputs','++','Position',[560 365 580 385]);
add_line(mdl,'Heading psi/1','Track chi/1','autorouting','on');
add_line(mdl,'Demux rad/1','Track chi/2','autorouting','on');

%% ---- North and East position (metres) ----
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos chi'], ...
    'Operator','cos','Position',[620 330 660 360]);
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin chi'], ...
    'Operator','sin','Position',[620 380 660 410]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain North'], ...
    'Gain','V_ms*pos_scale','Position',[690 330 750 360]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain East'], ...
    'Gain','V_ms*pos_scale','Position',[690 380 750 410]);
add_block('simulink/Continuous/Integrator', [mdl '/North'], ...
    'InitialCondition','0','Position',[780 330 810 360]);
add_block('simulink/Continuous/Integrator', [mdl '/East'], ...
    'InitialCondition','0','Position',[780 380 810 410]);
add_line(mdl,'Track chi/1','cos chi/1','autorouting','on');
add_line(mdl,'Track chi/1','sin chi/1','autorouting','on');
add_line(mdl,'cos chi/1','Gain North/1','autorouting','on');
add_line(mdl,'sin chi/1','Gain East/1','autorouting','on');
add_line(mdl,'Gain North/1','North/1','autorouting','on');
add_line(mdl,'Gain East/1','East/1','autorouting','on');

%% ---- Translation vector [North East Down], metres ----
add_block('simulink/Sources/Constant', [mdl '/Down'], ...
    'Value','-alt_m','Position',[780 430 840 450]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux T'], ...
    'Inputs','3','Position',[880 340 885 440]);
add_line(mdl,'North/1','Mux T/1','autorouting','on');
add_line(mdl,'East/1','Mux T/2','autorouting','on');
add_line(mdl,'Down/1','Mux T/3','autorouting','on');

%% ---- Rotation vector [roll pitch yaw], radians ----
add_block('simulink/Sources/Constant', [mdl '/Pitch thetae'], ...
    'Value','thetae','Position',[470 500 530 520]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux R'], ...
    'Inputs','3','Position',[880 480 885 560]);
add_line(mdl,'Demux rad/4','Mux R/1','autorouting','on');
add_line(mdl,'Pitch thetae/1','Mux R/2','autorouting','on');
add_line(mdl,'Heading psi/1','Mux R/3','autorouting','on');

%% ---- Temporary terminators (delete them when you connect the Airliner Pack) ----
add_block('simulink/Sinks/Terminator', [mdl '/Terminator T'], 'Position',[920 380 940 400]);
add_block('simulink/Sinks/Terminator', [mdl '/Terminator R'], 'Position',[920 510 940 530]);
add_line(mdl,'Mux T/1','Terminator T/1','autorouting','on');
add_line(mdl,'Mux R/1','Terminator R/1','autorouting','on');

%% ---- Optional: run in real time so the 20 s case takes about 20 s ----
try
    set_param(mdl,'EnablePacing','on','PacingRate','1');
catch
    disp('Pacing could not be set automatically: use Simulation > Pacing in the toolbar.');
end

save_system(mdl);

fprintf('\nVisualization path added to %s.\n', mdl);
fprintf('Next, add these three blocks by hand (library browser: Aerospace Blockset > Animation > Simulation 3D):\n');
fprintf('  1. Simulation 3D Scene Configuration : Scene source = Default Scenes, Scene name = Airport\n');
fprintf('  2. Simulation 3D Airliner Pack       : Body_T <- Mux T,  Body_R <- Mux R  (delete the two Terminators)\n');
fprintf('  3. Simulation 3D Aircraft            : Type = Airliner, Sample time = -1,\n');
fprintf('                                         Translation and Rotation <- Pack outputs\n');
fprintf('Then set pos_scale = 0 first (attitude only) and run.\n');
end