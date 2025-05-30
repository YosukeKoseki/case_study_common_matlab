function plant_data = save_data_for_machine_learning(Data)
    % Data: cell配列になっていてDNN用に整形したいSim / Expの元データ
    data = simplifyLogger(Data.log, Data.log.target);
    % plant_data.i = Data.log.target
    plant_data.input = data.controller;
    plant_data.q
    plant_data.p
    plant_data.v
    plant_data.w
end