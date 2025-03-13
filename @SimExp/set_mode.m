function set_mode(app)
Setting = app.initial_setting;
app.reset_app;
% run(app.mode);
if isfield(Setting,"do_calculation")
  app.do_calculation = @(A) Setting.do_calculation(A);
else
  app.agent = Setting.agent;
  app.N = length(Setting.agent);
  %app.NumLabel.Text = "Num : " + string(app.N);
  % app.PlantLabel.Text = ["Plant : "+  class(agent(1).plant)];
  % app.EstimatorLabel.Text = ["Estimator : "+class(agent(1).estimator), "Param : "+class(agent(1).parameter)];
  % app.SensorLabel.Text = ["Sensor : "+  class(agent(1).sensor)];
  % app.ReferenceLabel.Text = ["Reference : "+ class(agent(1).reference)];
  % app.ControllerLabel.Text = ["Controller : "+ class(agent(1).controller)];
  app.logger = Setting.logger;
  for i = 1:app.N
    app.agent(i).cha_allocation = app.set_cha_allocation(Setting.agent(i));
  end

  % if app.fExp
  %   app.flight_reference = agent.reference;
  %   app.flight_estimator = agent.estimator;
  %   app.flight_controller = agent.controller;
  %   app.flight_input_transform = agent.input_transform;
  %   app.agent.estimator = app.dummy_class(agent.estimator);
  %   app.agent.reference = app.dummy_class(agent.reference);
  %   app.agent.controller = app.dummy_class(agent.controller);
  % end
  if isfield(Setting,"env"); app.env = Setting.env; end
  if isfield(Setting,"in_prog_func"); app.in_prog = Setting.in_prog_func;     end
  if isfield(Setting,"post_func"); app.post = Setting.post_func;      end
end
app.time = Setting.time;
app.TimeSlider.Value = 0;
app.TimeSlider.Limits = [Setting.time.ts Setting.time.te];
app.TimeSlider.MajorTicks = Setting.time.ts:(Setting.time.te-Setting.time.ts)/5:Setting.time.te;
if isfield(Setting,"motive"); app.motive = Setting.motive; end
app.clear_axes;
app.TimeSliderLabel.Text = ["time : " + app.time.t];
end