function Controller= Controller_SIMPLE_MEC(dt)
% SIMPLE_MEC用補償ゲインの設定

% ↓非線形モデルに直接適用する方
% Controller.D_thrust = [100, 20]; % Sim, m=0.825
% Controller.D_thrust = [560, 20]; % Sim, m=0.825,0.9

% Controller.D_thrust = [100, 10]; % Exp, m=0.75
Controller.D_thrust = [0, 10]; % Exp, m=0.825
% Controller.D_thrust = [560, 20]; % Exp, m=0.825,0.9

Controller.D_roll   = [0, 0, 0, 0];
Controller.D_pitch  = [0, 0, 0, 0];
Controller.D_yaw    = [0, 0];


% ↓HLで線形化した各サブシステムに適用する方
Controller.D_z1 = [900, 20];
Controller.D_z2 = [500, 100, 40, 40];
Controller.D_z3 = [500, 100, 40, 40];
Controller.D_z4 = [1, 1];

% Controller.D_z1 = [1, 1];
% Controller.D_z2 = [1, 1, 1, 1];
% Controller.D_z3 = [1, 1, 1, 1];
% Controller.D_z4 = [1, 1];

Controller.dt = dt;
end
