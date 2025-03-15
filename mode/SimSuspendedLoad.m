clc
ts = 0; % initial time
% dt = 0.025; % sampling period
dt = 0.025; % sampling period
te = 25; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);

initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.pL = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];

motive = Connector_Natnet_sim(2, dt, 0); % imitation of Motive camera (motion capture system)
agent(1) = DRONE;
agent(1).parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent(1).plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));%id,dt,type,initial,varargin

agent(2) = DRONE;
agent(2).parameter = agent(1).parameter;
initial_load.p = initial_state.pL;
initial_load.v = [0;0;0];
initial_load.q = [0;0;0];
Model.type = "discrete";
Model.name = "discrete";
Model.id = 2;
load_setting.dt = dt;
load_setting.method = @(~,x,~)x; 
load_setting.dim = [6,6,0];
load_setting.state_list = ["p","q"];
load_setting.num_list = [3,3];
load_setting.initial = initial_load;
Model.param = load_setting;
agent(2).plant = MODEL_CLASS(agent(2),Model);
%agent(2).sensor.do = @(obj, varargin)[];
agent(2).estimator.do = @(obj, varargin)[];
agent(2).reference.do = @(obj, varargin)[];
agent(2).controller.do = @load_controller;
function result = load_controller(~,~,~,~,agent,~)
    agent(2).plant.set_state("p",agent(1).plant.state.pL,"q",agent(1).plant.state.pT);
    result = agent(2).plant.state;
end
motive.getData(agent);

agent(1).estimator.forload = FOR_LOAD(agent(1), Estimator_Suspended_Load(2));%[1,1+N]%for_loadで機体と牽引物の位置、姿勢をstateクラスに格納
Estimator = Estimator_EKF(agent(1),dt,MODEL_CLASS(agent(1),Model_Suspended_Load(dt, initial_state, 1,agent(1),1)), ["p", "q", "pL", "pT"]);
	Estimator.sensor_func = @EKF_muliti
function state = EKF_multi(self,param) 
r =self.sensor.result.rigid;
state = r(1);
state.pL = r(2).p;
state.pT = r(2).q;
end
agent(1).estimator.ekf = EKF(agent(1), Estimator );%expの流用
agent(1).estimator.result = agent(1).estimator.ekf.do(time,'a',logger,[],agent,1);
% agent(1).parameter.set("loadmass",0.3);
% agent(1).sensor = DIRECT_SENSOR(agent(1), 0.002*0);%motiveとforloadに書き換えると実験と同じ条件でできる
agent(1).sensor.motive = MOTIVE(agent(1), Sensor_Motive(1,0, motive));%荷物のも取ってこれるはず
agent(1).sensor.motive.do(time,'a',logger,[],agent,1);

agent(2).sensor.motive = MOTIVE(agent(1), Sensor_Motive(2,0, motive));%荷物のも取ってこれるはず
agent(2).sensor.motive.do(time,'a',logger,[],agent,1);
agent(1).reference.timevarying = TIME_VARYING_REFERENCE(agent(1),{"gen_ref_saddle",{"freq",10,"orig",[0;0;1],"size",[2,2,0.5]},"HL"});
agent(1).controller = HLC_SUSPENDED_LOAD(agent(1),Controller_HL_Suspended_Load(dt,agent(1)));
agent(1).controller.result.input = [(agent(1).parameter.loadmass*0+agent(1).parameter.mass)*agent(1).parameter.gravity;0;0;0];
run("ExpBase");
agent(1).cha_allocation.sensor = "motive";
agent(2).cha_allocation.sensor = "motive";
agent(1).cha_allocation.estimator = ["forload","ekf"];
agent(1).cha_allocation.reference = "timevarying";
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
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "plant.result.state.pL", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
end
function in_prog(app)
app.Label_2.Text = ["estimator : " + app.agent(1).estimator.result.state.get()];
end