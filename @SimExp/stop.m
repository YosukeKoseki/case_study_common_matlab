
function stop(app)
app.fStart = 0;
app.StartButton.Text = "Start";
app.Lamp.Color = [0 0 0];
app.LampLabel.Text = "Program Stop";
app.cha = "";
app.cha0 = "";
if ~isempty(app.update_timer)
  stop(app.update_timer);
  delete(app.update_timer);
  app.update_timer = [];
end
end