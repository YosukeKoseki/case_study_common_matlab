
function loop(app)
pause(0.1);
if app.fExp % experiment
  app.LampLabel.Text = ["== EXPERIMENT ==","Start! Press key code" + app.cha,"s : stop", "a : arming", "t : take-off", "f : fligh", "l : landing"];
  app.TimeSliderLabel.Text = ["..."];
  uiwait(app.UIFigure)
  start(app.update_timer);
  % Caution : N agents are not allowed.
  while app.fStart
    tStart = tic;
    drawnow
    if app.cha0 ~= app.cha % exec once per another key press
      switch app.cha
        case {'q',' '}
          app.StopProp;
          app.LampLabel.Text = "Quit the trial";
          app.Lamp.Color = [0 0 0];
          break;
        case 's'
          app.StopProp;
          app.LampLabel.Text = "Stop propellas";
          app.Lamp.Color = [0 1 0];
        case 'f'
          if strcmp(app.cha0,'t') % "flight" is allowed after "take-off"
            app.LampLabel.Text = "Flight";
            % app.Flight;
          end
        case 'l'
          app.LampLabel.Text = "Landing";
          % app.Landing;
        case 't'
          if strcmp(app.cha0,'a') % "take-off" is allowed after "arming"
            app.LampLabel.Text = "Take-off";
            % app.Takeoff;
          end
        case 'a'
          app.LampLabel.Text = "Arming";
          % app.Arming;
          app.Lamp.Color = [1 0 0];
      end
      app.cha0 = app.cha;
    end
    if ~isempty(app.motive);              app.motive.getData(app.agent);            end
    %app.TimeSlider.Value = app.time.t;
    app.do_calculation();
    app.time.dt = toc(tStart);
    app.time.t = app.time.t + app.time.dt;
  end
else % simulation
  app.LampLabel.Text = "Press any key to start. (except for 'q', 'space', 'enter')";
  uiwait(app.UIFigure)
  %start(app.update_timer);
  while app.fStart && (app.time.t < app.time.te)
    drawnow;
    if app.cha0 ~= app.cha
      switch app.cha
        case {'q',' '}
          app.LampLabel.Text = "Quit the trial";
          app.Lamp.Color = [1 0 0];
          break;
        otherwise
      end
      app.cha0 = app.cha;
    end
    if ~isempty(app.motive)
      app.motive.getData(app.agent);
    end
    app.do_calculation();
    app.TimeSlider.Value = app.time.t;
    app.TimeSliderLabel.Text = string(app.time.t);
    app.time.t = app.time.t + app.time.dt;
    if app.fDebug && app.time.t > app.time.dt % not active at the initial step
      app.in_prog(app);
    end
  end
end
app.stop;
end
