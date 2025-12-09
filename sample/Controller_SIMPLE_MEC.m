function Controller= Controller_SIMPLE_MEC(dt)
% SIMPLE_MEC用補償ゲインの設定

% ↓非線形モデルに直接適用する方
Controller.D_thrust = [100, 20];
Controller.D_roll   = [100, 20, 5, 1];
Controller.D_pitch  = [100, 20, 5, 1];
Controller.D_yaw    = [1, 1];


% ↓HLで線形化した各サブシステムに適用する方
Controller.D_z1 = [800, 20];
Controller.D_z2 = [100, 20, 5, 30];
Controller.D_z3 = [100, 20, 5, 30];
Controller.D_z4 = [1, 1];

% Controller.D_z1 = [1, 1];
% Controller.D_z2 = [1, 1, 1, 1];
% Controller.D_z3 = [1, 1, 1, 1];
% Controller.D_z4 = [1, 1];

Controller.dt = dt;
end
