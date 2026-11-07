%% setup_paths.m  - run once per MATLAB session from the repository root
% Adds every MATLAB sub-folder (flight_conditions, comparison, simulink) to the path
% so scripts can find F104_build_model / F104_add_visualization and the .slx models.
root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
fprintf('F-104 project paths added: %s\n', root);
