tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear ; clc;
userpath('clear');
end

%%
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

% % DRONEクラスの定義 % % % %
agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
% プラントモデル定義 ================================================================================================================================
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1)); % Model_Quat13

% デフォルト物理パラメータ(DRONE_PARAM.m準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
agent.plant.param(6) = 0.24; % 0.18<jx,jy<0.22ぐらいが良き frequency=5の時
agent.plant.param(7) = 0.24; % 同上
% 2~5,10~18はagent.plant.method='@roll_pitch_yaw_thrust_torque_physical_parameter_model'を使っている限り意味が無い
%===================================================================================================================================================
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});

run("ExpBase");
agent.cha_allocation.reference = "time_varying";
agent.controller.nominal = HLC(agent,Controller_HL(dt));
agent.controller.mec = SIMPLE_MEC(agent);
agent.cha_allocation.controller = ["nominal","mec"]; % cha_allocationにコントローラー登録
motive.getData(agent);

function dfunc(app)
LW = 1.5; % Linewidth 
FS = 20; % Fontsize
phase = "tfl";
% phase = "f";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase",phase, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({{1, "p", "er"}, {1, "controller.result.nominal_p", ""}},"phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS); % 位置: p_x,p_y,p_z
% app.logger.plot({{1, "q", "e"}, {1, "controller.result.nominal_q", ""}}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({{1, "v", "er"}, {1, "controller.result.nominal_v", ""}}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);% 速度: v_x, v_y, v_z
% app.logger.plot({{1, "w", "e"}, {1, "controller.result.nominal_w", ""}}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS); % 角速度: ω_roll, ω_ptich, ω_yaw
% app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",24); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({1, "controller.result.delta_input", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",24);

app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",8, "Linewidth",LW, "Fontsize",FS); % x-y軌跡
% app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS); % x-y-z軌跡


app.logger.plot({1, "p", "er"},"phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS); % 位置: p_x,p_y,p_z
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);% 速度: v_x, v_y, v_z
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",24); % 制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "controller.result.delta_input", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",24);
end