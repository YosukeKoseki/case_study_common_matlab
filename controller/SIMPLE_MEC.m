classdef SIMPLE_MEC < handle
    %SIMPLE_MEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   手チューニングによるPゲインで補償器を設計
    
    properties
        self
        result
        param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        D_thrust    % Δu_thrust補償ゲイン
        D_roll      % Δu_roll補償ゲイン
        D_pitch     % Δu_pitch補償ゲイン
        D_yaw       % Δu_yaw補償ゲイン
        x_pre       % 前時刻の状態
        pre_input   % 前時刻の制御入力
    end
    
    methods
        function obj = SIMPLE_MEC(self)
            obj.self = self;
            obj.param = self.parameter.get(obj.parameter_name);
            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);

            %-%-%-% 補償ゲイン設計 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            % obj.D_thrust = [1, 1];
            % obj.D_roll = [1, 1, 1, 1];
            % obj.D_pitch = [1, 1, 1, 1];
            % obj.D_thrust = [400, 20];
            % obj.D_roll = [400, 20, 10, 2];
            % obj.D_pitch = [400, 20, 10, 2];
            obj.D_thrust = [100, 20];
            obj.D_roll = [100, 20, 5, 1];
            obj.D_pitch = [100, 20, 5, 1];
            obj.D_yaw = [1, 1];
        end
        
        function result = do(obj, varargin)
            %-%-%-% ノミナル状態更新 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator")... % ループの最初はLoggingされていなくて，参照できないのを回避
            && length(varargin{3}.Data.agent.estimator.result)>=2
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.nominal_input; % LOGGERから前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERから前時刻の状態を取得
            end
            dt = varargin{1}.dt;
            dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param);
            x_nominal = obj.x_pre + dx*dt;
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.result.nominal_p = x_nominal(1:3);
            obj.result.nominal_q = x_nominal(4:6);
            obj.result.nominal_v = x_nominal(7:9);
            obj.result.nominal_w = x_nominal(10:12);

            % プラント値取得
            x_plant = obj.self.estimator.result.state.get; % 現時刻の推定値


            z = x_plant - x_nominal; % [p; q; v; w]
            du_thrust = obj.D_thrust*[z(3); z(9)];              % p_z; v_z
            du_roll = obj.D_roll*[z(2); z(8); z(4); z(10)];     % p_y; v_y; θ_roll; ω_roll
            du_pitch = obj.D_pitch*[z(1); z(7); z(5); z(11)];   % p_x; v_x; θ_pitch; ω_pitch
            du_yaw = obj.D_yaw*[z(6); z(12)];                   % θ_yaw; ω_yaw
            obj.result.delta_input = -1*[du_thrust; du_roll; du_pitch; du_yaw]; % 符号注意！！
            % obj.result.delta_input = [0;0;0;0];

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input; % ノミナル入力を保存
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
            
            disp(obj.result.delta_input')
        end
    end
end

