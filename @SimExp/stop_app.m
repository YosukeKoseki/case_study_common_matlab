function stop_app(app,STR)
arguments
  app
  STR = ""
end
% Normal finish
app.fStart = 0;
app.isReady = false;
app.StartButton.Text = "Start: " + app.cha + " " + STR;
app.Lamp.Color = [0 0 0];
app.LampLabel.Text = "Program Stop";
app.cha = "s";
app.cha0 = "s";
if ~isempty(app.update_timer)
  stop(app.update_timer);
  delete(app.update_timer);
  app.update_timer = [];
end
end