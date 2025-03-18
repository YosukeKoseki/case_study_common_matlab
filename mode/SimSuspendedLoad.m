clc
ts = 0; % initial time
% dt = 0.025; % sampling period
dt = 0.025; % sampling period
te = 25; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(2, size(ts:dt:te, 2), 0, [],[]);
motive = Connector_Natnet_sim(2, dt, 0); % imitation of Motive camera (motion capture system)

% drone plant setting


agent(2) = DRONE;
agent(2).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
%agent(2).parameter.set("loadmass", 0.1)
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.pL = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
agent(2).plant = MODEL_CLASS(agent(2),Model_Suspended_Load(dt, initial_state,2,agent(2)));%id,dt,type,initial,varargin


% Load plant setting
agent(1) = DRONE;
agent(1).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
initial_load.p = initial_state.pL;
initial_load.q = [0;0;0];
Model.type = "discrete";
Model.name = "discrete";
Model.id = 1;
load_setting.dt = dt;
load_setting.method = @(~,u,~)u; % controller.result.input をそのまま次の時刻の状態に設定
load_setting.dim = [6,6,0]; % state, output, input
load_setting.state_list = ["p","q"];
load_setting.num_list = [3,3]; % dim of p, q
load_setting.initial = initial_load;
Model.param = load_setting;
agent(1).plant = MODEL_CLASS(agent(1),Model); % motiveがplantのp, qを取ってくるため plantを設定

% getDataするためにはplantを先に設定しておく必要がある
motive.getData(agent);

% Load setting
agent(1).estimator.do = @(obj, varargin)[];% dummy%@(varargin)struct('state',varargin{5}(1).plant.state);% dummy
agent(1).reference.do = @(obj, varargin)[];% dummy
agent(1).controller.do = @load_controller;% plant の状態に設定するため 
function result = load_controller(~,~,~,~,agent,~)
    agent(1).plant.set_state("p",agent(2).plant.state.pL); % 統合モデルのpL を pとして設定
    result.input = agent(1).plant.state; % p = pL となるように設定
end
agent(1).sensor.motive = MOTIVE(agent(1), Sensor_Motive(2,0, motive));%荷物のも取ってこれるはず
% agent(1).sensor.motive.do(time,'a',logger,[],agent,1);

% drone setting
Estimator = Estimator_EKF(agent(2),dt,MODEL_CLASS(agent(2),Model_Suspended_Load(dt, initial_state, 2,agent(2),"Load_mL_HL")), ["p", "q", "pL", "pT"]);
Estimator.sensor_func = @EKF_sensor_multi_rigid;
function state = EKF_sensor_multi_rigid(self,~) 
r =self.sensor.result.rigid; % motive情報
d = r(1).p - r(2).p; % 機体から見たload位置
state = [r(2).p;Quat2Eul(r(2).q); r(1).p; d/norm(d)]; % p, q, pL, pT
end
agent(2).estimator.ekf = EKF(agent(2), Estimator );%expの流用
% agent(2).estimator.result = agent(2).estimator.ekf.do(time,'a',logger,[],agent,2);
% agent(2).parameter.set("loadmass",0.3);
% agent(2).sensor = DIRECT_SENSOR(agent(1), 0.002*0);%motiveとforloadに書き換えると実験と同じ条件でできる
agent(2).sensor.motive = MOTIVE(agent(2), Sensor_Motive(1,0, motive));
% agent(2).sensor.motive.do(time,'a',logger,[],agent,2);


agent(2).reference.timevarying = TIME_VARYING_REFERENCE(agent(2),{"gen_ref_saddle",{"freq",10,"orig",[0;0;1],"size",[2,2,0.5]},"HL"});
agent(2).controller = HLC_SUSPENDED_LOAD(agent(2),Controller_HL_Suspended_Load(dt,agent(2)));
% agent(2).controller.result.input = [(agent(2).parameter.loadmass*0+agent(2).parameter.mass)*agent(2).parameter.gravity;0;0;0];
run("ExpBase");
agent(2).cha_allocation.sensor = "motive";
agent(2).cha_allocation.estimator = "ekf";
agent(2).cha_allocation.reference = "timevarying";
agent(1).cha_allocation.sensor = "motive";
agent(1).cha_allocation.l = [];
agent(1).cha_allocation.t = [];

%%
% clc
% for i = 1:time.te
%     if i < 20 || rem(i, 10) == 0, i, end
%     agent(1).sensor.do(time, 'f');
%     agent(1).estimator.do(time, 'f');
%     agent(1).reference.do(time, 'f');
%     agent(1).controller.do(time, 'f',0,0,agent,1);
%     agent(1).plant.do(time, 'f');
%     logger.logging(time, 'f', agent);
%     time.t = time.t + time.dt;
%     %pause(1)
% end

%%
% logger.plot({1,"plant.result.state.pL","p"},{1,"input",""})
% logger.plot({1,"plant.result.state.pL","p"})
%%

function post(app)
app.logger.plot({1, "controller.result.xd1:3", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "plant.result.state.pL", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "p", "pr"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
end
function in_prog(app)
app.Label_2.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end