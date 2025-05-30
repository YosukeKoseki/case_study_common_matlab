function plant_data = save_data_for_machine_learning(Data)
    % Data: cell配列になっていてDNN用に整形したいSim / Expの元データ
    data = simplifyLogger(Data.log, Data.log.target);
    flight = 102;
    firstindex = find(data.phase == flight, 1, 'first')
    lastindex = find(data.phase == flight, 1, 'last')
    % plant_data.i = Data.log.target
    plant_data.input = data.controller.input;
    plant_data.q = data.estimator.q;
    plant_data.p = data.estimator.p;
    plant_data.v = data.estimator.v;
    plant_data.w = data.estimator.w;
end