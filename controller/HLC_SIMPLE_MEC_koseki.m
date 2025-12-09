classdef HLC_SIMPLE_MEC_koseki < handle
    %SIMPLE_MEC
    %   クアッドコプター用モデル誤差補償器(MEC)のプログラム
    %   HLCと合体している
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
        F1          % 線形化時に使う第１層のHLゲイン
        F2
        F3
        F4
    end
    
    methods
        function obj = HLC_SIMPLE_MEC_koseki(self, param, HLgain)
            obj.self = self;
            obj.physical_param = self.parameter.get(obj.parameter_name);
            obj.result.nominal_p = zeros(3,1);
            obj.result.nominal_q = zeros(3,1);
            obj.result.nominal_v = zeros(3,1);
            obj.result.nominal_w = zeros(3,1);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input_HL = zeros(self.estimator.model.dim(2),1);
            obj.result.nominal_input_HL = zeros(self.estimator.model.dim(2),1);
            obj.result.input_HL = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
            obj.result.pre_dv_z1 = 0; % 前時刻のz1サブシステムの補償仮想入力
            size2 = zeros(2,1);
            size4 = zeros(4,1);
            obj.result.zone_p = size2;
            obj.result.ztwo_p = size4;
            obj.result.zthree_p = size4;
            obj.result.zfour_p = size2;
            obj.result.zone_n = size2;
            obj.result.ztwo_n = size4;
            obj.result.zthree_n = size4;
            obj.result.zfour_n = size2;
            obj.F1 = HLgain.F1;
            obj.F2 = HLgain.F2;
            obj.F3 = HLgain.F3;
            obj.F4 = HLgain.F4;

            %-%-%-% 補償ゲイン設計 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            obj.D_z1 = param.D_z1;
            obj.D_z2 = param.D_z2;
            obj.D_z3 = param.D_z3;
            obj.D_z4 = param.D_z4;

            disp('目標軌道と補償入力を表示します')
            disp('x,y,z, dthrust, dtau_roll, dtau_pitch, dtau_yaw')
        end
        
        function result = do(obj, varargin)
            %-%-%-% ノミナル状態更新 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%
            pre_dv_z1 = 0;
            if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator")... % ループの最初はLoggingされていなくて，参照できないのを回避
            && length(varargin{3}.Data.agent.estimator.result)>=2
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.nominal_input; % LOGGERから前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{end}.state.get; % LOGGERから前時刻の状態を取得
                pre_dv_z1 = varargin{3}.Data.agent.controller.result{end}.pre_dv_z1; % 前時刻のz1サブシステムの補償仮想入力を取得
            end
            dt = varargin{1}.dt;
            dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.physical_param);
            x_nominal = obj.x_pre + dx*dt;
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-
            obj.result.nominal_p = x_nominal(1:3);
            obj.result.nominal_q = x_nominal(4:6);
            obj.result.nominal_v = x_nominal(7:9);
            obj.result.nominal_w = x_nominal(10:12);

            % % ノミナルの線形化 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-
            ref = obj.self.reference.result;
            xd = ref.state.xd;
            xd = [xd; zeros(20 - size(xd, 1), 1)]; % 足りない分は一旦０で埋める．
    
            qn_quat = Eul2Quat(obj.result.nominal_q);   % オイラー角 → クォータニオンに変換
            qn_rotm = RodriguesQuaternion(qn_quat);     % クォータニオン → 回転行列に変換
            Rb0 = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]));
            model = obj.self.estimator.result;
            xp = [R2q(Rb0'*model.state.getq("rotmat"));Rb0'*model.state.p;Rb0'*model.state.v;model.state.w]; % [q, p, v, w]に並べ替え
            xn = [R2q(Rb0' * qn_rotm); Rb0' * obj.result.nominal_p; Rb0' * obj.result.nominal_v; obj.result.nominal_w]; % [q, p, v, w]に並べ替え
            xd(1:3) = Rb0' * xd(1:3);
            xd(4) = 0;
            xd(5:7) = Rb0' * xd(5:7);
            xd(9:11) = Rb0' * xd(9:11);
            xd(13:15) = Rb0' * xd(13:15);
            xd(17:19) = Rb0' * xd(17:19);

            % x_nを線形化して z1_n, z2_n, z3_n, z4_nを生成する
            z1_n = Z1(xn, xd', obj.physical_param);
            vf_tmp = Vf(xn, xd',obj.physical_param,obj.F1);
            z2_n = Z2(xn, xd', vf_tmp, obj.physical_param);
            z3_n = Z3(xn, xd', vf_tmp, obj.physical_param);
            z4_n = Z4(xn, xd', vf_tmp, obj.physical_param);
            %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-


            % 線形化されたプラント値取得
            z1_p = Z1(xp, xd', obj.physical_param);
            vf_n = Vf(xp, xd', obj.physical_param, obj.F1);     % ノミナルコントローラの第１層仮想入力 vf = [vz1, dvz1, ddvz1, dddvz1]
            z2_p = Z2(xp, xd', vf_n, obj.physical_param);
            z3_p = Z3(xp, xd', vf_n, obj.physical_param);
            z4_p = Z4(xp, xd', vf_n, obj.physical_param);
            vs_n = Vs(xp,xd',vf_n,obj.physical_param,obj.F2,obj.F3,obj.F4); % ノミナルコントローラの第2,3,4層仮想入力 vs = [vz2, vz3, vz4]
            obj.result.nominal_input = Uf(xp, xd', vf_n, obj.physical_param) + Us(xp, xd', vf_n, vs_n', obj.physical_param);
            
            % 補償入力生成 %-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-%-
            % TODO z, znの行列サイズ要確認
            dv_z1 = obj.D_z1*(z1_p - z1_n);
            obj.result.pre_dv_z1 = dv_z1;
            ddv_z1 = (dv_z1 - pre_dv_z1)/ dt; % dv_z1 の１階微分値
            dv_z2 = obj.D_z2*(z2_p - z2_n);
            dv_z3 = obj.D_z3*(z3_p - z3_n);
            dv_z4 = obj.D_z4*(z4_p - z4_n);

            % dv_z1 = 0; % MECを無しにしたいときに使う
            % ddv_z1 = 0;
            % dv_z2 = 0;
            % dv_z3 = 0;
            % dv_z4 = 0;

            %TODO 仮想入力dv -> 実入力duへの変換
            vf_a = vf_n - [dv_z1, ddv_z1, 0, 0];
            vs_a = vs_n - [dv_z2, dv_z3, dv_z4];
            obj.result.input = Uf(xp, xd', vf_a, obj.physical_param) + Us(xp, xd', vf_a, vs_a', obj.physical_param);
            
            % 線形化ダイナミクス関連保存
            obj.result.zone_p = z1_p;
            obj.result.ztwo_p = z2_p;
            obj.result.zthree_p = z3_p;
            obj.result.zfour_p = z4_p;
            obj.result.zone_n = z1_n;
            obj.result.ztwo_n = z2_n;
            obj.result.zthree_n = z3_n;
            obj.result.zfour_n = z4_n;
            obj.result.nominal_input_HL = [vf_n(1); vs_n'];
            obj.result.delta_input_HL   = -1*[dv_z1; dv_z2; dv_z3; dv_z4];
            obj.result.input_HL         = [vf_a(1); vs_a'];
            % disp([ref.state.p', obj.result.delta_input_HL'])
            disp([model.state.p', (obj.result.input - obj.result.nominal_input)'])

            result = obj.result;
        end
    end
end

