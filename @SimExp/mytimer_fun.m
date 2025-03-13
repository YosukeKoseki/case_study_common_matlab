function mytimer_fun(app, obj, event)
% Timer で指定するコールバック関数
if app.t0 == app.time.t && app.time.t > 0
  app.Label_2.Text = ["","","===========  Emergency stop!  ========="];
  app.StopProp;
  app.fStart = 0;
end
app.t0 = app.time.t;
end