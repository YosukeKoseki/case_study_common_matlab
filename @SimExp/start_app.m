
% Button pushed function: StartButton
function start_app(app, event)
if ~isempty(timerfind)
    stop(timerfind)
    delete(timerfind)
    app.update_timer = [];
end
app.UIFigure.WindowKeyPressFcn = @app.keyPressFunc;
if app.fStart % to stop
    app.stop_app();
else % to start
    app.StartButton.Text = "Stop";
    app.Lamp.Color = [0 1 0];

    if ~app.isReady
        for cha = ['a','t','f','l']
            app.fStart = 1;
            app.cha = cha;
            app.loop();
        end
        app.isReady = true;
        app.fStart = 1;
        app.LampLabel.Text = "Ready!";
    end
    app.update_timer = timer('Period', app.PInterval,... % poling interval
        'ExecutionMode', 'fixedSpacing', ... % execution mode
        'TasksToExecute', Inf, ... % trial number
        'TimerFcn', @app.timer_callback); % callback function
    app.loop();
end
end
