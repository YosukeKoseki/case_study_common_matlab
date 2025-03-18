function timer_callback(app, obj, event)
% Timer で指定するコールバック関数
if app.t0 == app.time.t && app.time.t > app.time.ts 
  % 開始後にloop内の時間更新されていない場合
  app.LampLabel.Text = "===  Emergency stop! ===";
  app.StopProp;
  app.stop_app("EMG");
end
app.t0 = app.time.ts;
end