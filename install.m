%% Installation file. Adds local folders to path.

fprintf('Adding CoGoV folders to Matlab path... ')

%% add CG/ with subfolders (Supervision Scheme algorithms)
addpath(genpath(fullfile(pwd,'CG')));

%% add util/ with subfolders
addpath(genpath(fullfile(pwd,'util')));

%% add marine_vehicle/ with subfolders (models and examples)
addpath(genpath(fullfile(pwd,'marine_vehicle')));

fprintf('done.\n')
disp('Type "savepath" if you wish to store the changes.')
% savepath;