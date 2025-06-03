% Data/Exp_data の全てのmatファイルを学習用フォルダData/learning_dataに格納するようにしたい！！
% 2025.06.02


function plant_data = save_data_for_machine_learning(Data, phase)
% 任意のフェーズをdouble型に変換する関数
    % Data: cell配列になっていてDNN用に整形したいExpの元データ
    % phase: 'f', 'tf', 'atf', 'fl', 'tfl', 'atfl'から選択可能
    % 'f' -> flightフェーズのみ(default)
    % 'tfl' -> takeoff + flight + landingフェーズ
    arguments
        Data;
        phase = 'f';
    end

    if Data.log.fExp ~= 1
        error('Cannot use Sim data')
    end

    data = simplifyLogger(Data.log, Data.log.target);
    switch phase
        % arming  -> 97
        % takeoff -> 116
        % flight  -> 102
        % landing -> 108
        case 'tf'
            phase = 'takeoff + flight';
            firstindex = find(data.phase == 116, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
        case 'atf'
            phase = 'arming + takeoff + flight';
            firstindex = find(data.phase == 97, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
        case 'fl'
            phase = 'flight + landing';
            firstindex = find(data.phase == 102, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        case 'tfl'
            phase = 'takeoff + flight + landing';
            firstindex = find(data.phase == 116, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        case 'atfl'
            phase = 'arming + takeoff + flight + landing';
            firstindex = find(data.phase == 97, 1, 'first');
            lastindex = find(data.phase == 108, 1, 'last');
        otherwise
            phase = 'flight only';
            firstindex = find(data.phase == 102, 1, 'first');
            lastindex = find(data.phase == 102, 1, 'last');
    end

    fprintf('selected phase: %s', phase)
    % plant_data.i = Data.log.target
    plant_data.input = data.controller.input(:,firstindex:lastindex);
    plant_data.q = data.estimator.q(:,firstindex:lastindex);
    plant_data.p = data.estimator.p(:,firstindex:lastindex);
    plant_data.v = data.estimator.v(:,firstindex:lastindex);
    plant_data.w = data.estimator.w(:,firstindex:lastindex);
    
    % 確認用
    time = data.t(firstindex:lastindex);
    plot(time, plant_data.p(1,:), time, plant_data.p(2,:), time, plant_data.p(3,:))
    hold on
    plot(time, data.reference.p(1,firstindex:lastindex), '--', ...
        time, data.reference.p(2,firstindex:lastindex), '--', ...
        time, data.reference.p(3,firstindex:lastindex), '--')
    xlabel('t [s]')
    ylabel('position [m]')
    legend('xp', 'yp', 'xp', 'xr', 'yr', 'zr')
    title(['Selected phase: ',phase])
    grid on
    ax = gca;
    ax.FontSize = 12;
end