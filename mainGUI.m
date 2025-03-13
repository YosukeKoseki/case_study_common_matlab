%% Initialize settings
% set path
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');
%%
% each method's arguments : app.time,app.cha,app.logger,app.env,app.agent,i
clc
SimBaseMode = ["SimVoronoi", "SimHL","SimPointMass", "SimVehicle", "SimSuspendedLoad", "SimFHL", "SimFHL_Servo", "SimLiDAR", "SimFT", "SimEL", "SimMPC_Koopman"];
ExpBaseMode = ["ExpTestMotiveConnection", "ExpHL", "ExpFHL", "ExpFHL_Servo", "ExpFT", "ExpEL", "ExpMPC_Koopman"];
Setting.fExp = 0;
Setting.fDebug = 1; % 1: active : for debug function
Setting.PInterval = 0.6; % sec : poling interval for emergency stop
% run("SimVoronoi");
run("SimHL");
Setting.agent = agent;
Setting.logger = logger;
Setting.time = time;
if exist("motive","var"); Setting.motive = motive;end
if exist("env","var"); Setting.env = env;end
if exist("in_prog_func","var"); Setting.in_prog_func = in_prog_func;end
if exist("post_func","var"); Setting.post_func = post_func;end
gui = SimExp(Setting);
% gui = OSimExp_converted(Setting);