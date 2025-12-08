classdef HLC_SIMPLE_MEC < handle
    %SIMPLE_MEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   ！！！controller.nominalにFUNCTIONAL_HLCを設定する必要あり！！！
    %   手チューニングによるPゲインで補償器を設計
    
    properties
        self
        result
        physical_param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
        D_z1        % z1(=z)サブシステム 補償ゲイン
        D_z2        % z2(=x)サブシステム 補償ゲイン
        D_z3        % z3(=y)サブシステム 補償ゲイン
        D_z4        % z4(=yaw)サブシステム 補償ゲイン
        x_pre       % 前時刻の状態
        pre_input   % 前時刻の制御入力
    end
    
    methods
        function obj = HLC_SIMPLE_MEC(self, param)
            obj.self = self;
            obj.physical_param = self.parameter.get(obj.parameter_name);
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
            obj.D_z1 = param.D_z1;
            obj.D_z2 = param.D_z2;
            obj.D_z3 = param.D_z3;
            obj.D_z4 = param.D_z4;
        end
        
        function result = do(obj, varargin)
            %-%-%-% ノミナル状態更新 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator")... % ループの最初はLoggingされていなくて，参照できないのを回避
            && length(varargin{3}.Data.agent.estimator.result)>=2
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.nominal_input; % LOGGERから前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERから前時刻の状態を取得
            end
            dt = varargin{1}.dt;
            dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.physical_param);
            x_nominal = obj.x_pre + dx*dt;
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.result.nominal_p = x_nominal(1:3);
            obj.result.nominal_q = x_nominal(4:6);
            obj.result.nominal_v = x_nominal(7:9);
            obj.result.nominal_w = x_nominal(10:12);

            % % ノミナルの線形化
            ref = obj.self.reference.result;
            xd = ref.state.xd;
            xd = [xd; zeros(20 - size(xd, 1), 1)]; % 足りない分は０で埋める．
            %TODO x_nominalを線形化して，zn1,zn2,zn3,zn4を生成する
            zn1 = 
            zn2 = 
            zn3 = 
            zn4 = 

            % 線形化されたプラント値取得
            x_plant = obj.self.controller.nominal.result.state.get; % 現時刻の推定値
            z1 = obj.self.controller.nominal.result.z1;
            z2 = obj.self.controller.nominal.result.z2;
            z3 = obj.self.controller.nominal.result.z3;
            z4 = obj.self.controller.nominal.result.z4;

            %TODO z1~4, zn1~4の行列サイズ要確認
            dv_z1 = obj.D_z1*(z1 - zn1);
            dv_z2 = obj.D_z2*(z2 - zn2);
            dv_z3 = obj.D_z3*(z3 - zn3);
            dv_z4 = obj.D_z4*(z4 - zn4);

            %TODO 仮想入力dv -> 実入力duへの変換
            du1 = 
            du2 = 
            du3 = 
            du4 = 
            obj.result.delta_input = -1*[du1; du2; du3; du4]; % 符号注意！！
            % obj.result.delta_input = [0;0;0;0];

            obj.result.nominal_input = varargin{5}.controller.nominal.result.input; % ノミナル入力を保存
            obj.result.input = obj.result.nominal_input + obj.result.delta_input;
            result = obj.result;
            
            disp(obj.result.delta_input')
        end
    end
end

