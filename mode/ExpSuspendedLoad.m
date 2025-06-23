clc
ts = 0; % initial time
dt = 0.025;%0.025; % sampling period
% dt=0.03%粉砕
te = 10000; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
 % logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);%今までのやつ
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);

motive = Connector_Natnet('192.168.1.4'); % connect to Motive
motive.getData([], []); % get data from Motive
Drone = motive.result.rigid(1);
Load  = motive.result.rigid(2);
initial_state.p = Drone.p;
initial_state.q = Drone.q;
initial_state.pL = Load.p;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
%=推定方法を変える場合==========================================================================
%-拡張質量システム：
% Model_Suspended_Load(dt,initial,id,agent,isEstLoadMass):isEstLoadMass=1
%=============================================================================================

agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("cableL",0.99);%0.992,0.647,p0.613,0.460
agent.parameter.set("loadmass",0.05);%0.0968);%0.968
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM7"));%有線プロポ
agent.sensor.motive = MOTIVE(agent, Sensor_Motive([1,2],0, motive)); % rigid_id,initial_yaw_angle,motive
agent.estimator.ekf = EKF(agent, Estimator_EKF(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
    ["p", "q", "pL", "pT"],"sensor_func",@sensor_func));%expの流用
function y = sensor_func(self,~)
p = self.sensor.result.state.get('p');
q = self.sensor.result.state.getq('3');
pL = self.sensor.result.state(2).get('p');
pT = (pL - p);
pT = pT/vecnorm(pT);
y = [p;q;pL;pT];
end
agent.reference.timevarying = TIME_VARYING_REFERENCE(agent,...
    {"gen_ref_saddle",{"freq",12,"orig",[0;0;0.5],"size",[1,1,0.2*0]*1},"HL"});
agent.controller = HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone()); % 推力からスロットルに変換
run("ExpBase");
agent.cha_allocation.sensor = "motive";
agent.cha_allocation.estimator = "ekf";
agent.cha_allocation.f.reference = "timevarying";
%%

function post(app)
app.logger.plot({1, "controller.result.xd1:3", ""},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);

% 刻み時間描画
dt = diff(app.logger.Data.t(1:find(app.logger.Data.phase==0,1,'first')-1));
t = app.logger.data(0,'t',[]);
figure(100)
plot(t(1:end-1),dt);
hold on
yline(0.025,"LineWidth",0.5)
ylim([0 0.05])
hold off
grid on
legend("dt","upper limit")

% Graphplot(app)
end
function in_prog(app)
app.TextArea.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end