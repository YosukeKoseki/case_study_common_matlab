ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
% post_func = @(app) post(app); % function working at the "draw button" pushed.
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0); % [x, y], 1, 1, z
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
% model_file = '2025-10-08_exp_ob26_code00_randompp';
model_file = 'KMPCsim.mat';
%%
agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;1.0],"size",[1,1,0.3]},"HL"});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",8,"orig",[0;0;0.6],"size",[0,0,0]},"HL"});
agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",1.0},"HL"});
%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
agent.controller.kmpc = MPC_CONTROLLER_K(agent,Controller_MPC_K(dt,model_file,agent));
run("ExpBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = "kmpc";
function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"fig_num", 2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "q", "e"},  "fig_num",4,"xrange",[app.time.ts,app.time.te]); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "w", "e"},  "fig_num",5, "xrange",[app.time.ts,app.time.te]); % 角速度: ω_roll, ω_ptich, ω_yaw

app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 6,"phase",'tfl', "color",0);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.input_kmpc", ""}, "fig_num", 5,"phase",'f',"color",0);
% app.logger.plot({1, "controller.result.hlc", ""},{1, "controller.result.kmpc", ""},"fig_num", 6,"phase","f");
end