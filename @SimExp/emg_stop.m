function emg_stop(app, obj, event)
% Timer で指定するコールバック関数
if app.t0 == app.time.t && app.time.t > 0 % 開始後に時間更新されていない場合
  app.Label_2.Text = ["","","===========  Emergency stop!  ========="];
  app.StopProp;
  app.fStart = 0;
end
app.t0 = app.time.t;
end