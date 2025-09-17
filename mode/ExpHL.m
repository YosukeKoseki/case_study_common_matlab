ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.100.4'); % connect to Motive(Simと異なる部分)
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

% % DRONEクラスの定義 % % % %
agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM4")); % (Simと異なる部分)
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",1.0},"HL"});
agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,{"case_study_trajectory",{"freq",10,"init",[0;0;1]},"HL"});
agent.controller = HLC(agent,Controller_HL(dt));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % (Simと異なる部分)

run("ExpBase");
agent.cha_allocation.reference = "timevarying";

function post(app)
LW = 1.5; % Linewidth 
FS = 20; % Fontsize
phase = "tfl";
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase",phase, "Linewidth",LW, "Fontsize",FS);
app.logger.plot({1, "p", "er"},"phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS); % 位置: p_x,p_y,p_z
app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS);% 速度: v_x, v_y, v_z
app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input", ""}, "phase",phase, "fig_num",6, "Linewidth",LW, "Fontsize",FS); % MATLAB内での制御入力: Thrust, roll, pitch, yaw
app.logger.plot({1, "inner_input1:4", ""}, "phase",phase, "fig_num",7, "Linewidth",LW, "Fontsize",FS); % プロポ内での制御入力: Thrust, roll, pitch, yaw

app.logger.plot({1, "p1-p2", "er"}, "phase",phase, "color", 0, "fig_num",8, "Linewidth",LW, "Fontsize",FS); % x-y軌跡
app.logger.plot({1, "p1-p2-p3", "er"}, "phase",phase, "color", 0, "fig_num",9, "Linewidth",LW, "Fontsize",FS); % x-y-z軌跡
end
function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end