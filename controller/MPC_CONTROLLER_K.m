classdef MPC_CONTROLLER_K< handle

    properties
        options % QP
        param
        current_state
        input
        state
        const
        reference
        fRemove
        model
        result
        self
        sigma
        tss
    end
    properties
        % よく使うパラメータはobj.○○とする
        modelf
        modelp
        P % drone parameter
        N % 現時刻のパーティクル数
        H % horizon
        weight
        koopman
        qpparam % 二次計画法QPのパラメータ
        previous_input % 前時刻入力
        gen_beq
        removeN
        survive
        removeX
        flag
        reinput
        reEva
        StageStateSTLsum
        STL_period = [2,4]
        quadH
        quadf
        sw
        drf
        act
    end
    methods
        function obj = MPC_CONTROLLER_K(self, param)
            %-- 変数定義
            obj.self = self; % agent
            obj.param = param; % param = Controller_MPC_HLMC.mで設定したパラメーター
            %% flag defination
            obj.modelf = obj.self.plant.method;
            obj.P = obj.self.parameter.get(); % ドローンのパラメータ（質量，ロータ間距離，慣性モーメントなど）
            obj.N = param.particle_num; % サンプル数
            obj.H = param.H; % ホライズン
            % 重みの配列サイズ変換
            obj.weight = param.weight; % 重みを変数に保存
            obj.weight.stagestate = blkdiag(obj.weight.P, obj.weight.Q, obj.weight.V, obj.weight.W); % blkdiagで配列同士を結合
            obj.weight.terminalstate = blkdiag(obj.weight.Pf, obj.weight.Qf, obj.weight.Vf, obj.weight.Wf);
            obj.weight.input = param.weight.R;  % 目標入力
            obj.weight.preinputdif = param.weight.RP; % 前ステップとの入力
            % 入力の初期化
            obj.result.input = obj.param.ref_input; % 目標入力 初期時刻にresultを定義しておかないと実行時にエラー出る
            obj.input = obj.param.input; %入力関連のみ
            obj.input.pre_u = repmat(obj.result.input,1,obj.H); % 前入力
            obj.input.var = repmat(obj.param.ref_input,obj.H,1);
            obj.result.Bestcost_STL = 0;
            obj.input.sigma = param.input.Initsigma;
            % MPCパラメータ初期化 = メモリの確保
            obj.result.bestx(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.result.besty(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.result.bestz(1, :) = repmat(obj.input.Bestcost_now(1), obj.param.H, 1); % - 制約外は前の評価値を引き継ぐ
            obj.state.state_data = zeros(obj.param.state_size,obj.H, obj.N);
            obj.result.Evaluationtra = zeros(obj.N, 2);
            obj.result.pre_u = obj.input.pre_u;
            % obj.input.mu = param.ref_input;
            % A, B行列定義 z, x, y, yawの順番ベクトル化 speical defination for koopman
            obj.koopman = param.koopman;
            C = repmat({obj.koopman.C}, 1, obj.H);
            obj.koopman.ExC = blkdiag(C{:});
            [obj.koopman.ExA,obj.koopman.ExB] = ExtendedCoefficientMatrix({obj.koopman.A,obj.koopman.B,obj.H,param.state_size}); % 一括計算 2025/1/21確認
            %%Koopman予測に基づく拡張行列  　
            obj.result.bestcost = obj.input.Bestcost_now;
        end
        %-- main()的な
        function result = do(obj,varargin)
            time = varargin{1};
            phase = varargin{2};
            obj.param.t = time.t;
            obj.current_state = obj.self.estimator.result.state.get(); % 現在状態の取得
            obj.result2input();
            obj.state.current = obj.param.F([obj.current_state; obj.input.pre_u(:,1,1)]);
            obj.state.ref = obj.generate_reference(); % vararginのrefをHorizonに拡張
            result= obj.controller_KMC(varargin);
            % disp('controller: KMC,  phase: ');
            % disp(phase);
            obj.show();
        end
        function result = controller_KMC(obj,varargin)
            obj.param.t = varargin{1}{1}.t; % 現在時刻
            obj.param.te = varargin{1}{1}.te; % 終了時間(default : 10s)
            obj.K_MPC();%qp
            result = obj.result;
        end
        function K_MPC(obj)
            n = size(obj.state.current,1); % number of observables
            %qp def
            Q = blkdiag(kron(eye(obj.param.H-1),blkdiag(obj.weight.stagestate,0*eye(n-12))),blkdiag(obj.weight.terminalstate,0*eye(n-12)));
            R = kron(eye(obj.param.H),obj.weight.input);
            RP = kron(eye(obj.param.H),obj.weight.preinputdif);
            Xr = reshape([obj.state.ref(1:12,:);zeros(n-12,obj.param.H)],[],1);
            Ur = reshape(obj.state.ref(13:16,:),[],1);
            [obj.quadH,obj.quadf]=obj.gen_Hf(obj.koopman.ExA,obj.koopman.ExB,obj.state.current,Q,R,RP,Xr,Ur,obj.input.var);
            A = []; b = [];
            Aeq = []; beq = [];
            lb = repmat(obj.param.input_min,1,obj.param.H);
            ub = repmat(obj.param.input_max,1,obj.param.H);
            obj.options = optimset('Display', 'off');
            [var,fval,eflag,~,~] = quadprog(obj.quadH,obj.quadf,A,b,Aeq,beq,lb,ub,[],obj.options);
            if eflag ~= 1
                disp(['Warning: Quadprog failed to find a solution. eflag = ', num2str(eflag)]);
            end
            obj.result.input =var(1:4, 1); % 算出された入力
            obj.result.eflag = eflag;
            obj.result.var = var;
            obj.result.Bestcost_pre = obj.result.bestcost;
            obj.result.bestcost = [fval;0];
            obj.input.pre_u = obj.result.input;
            obj.result.pre_u = obj.input.pre_u;

        end
        function [H,f] = gen_Hf(obj,A,B,x0,Q,R,Rp,Xr,Ur,Up)
            % calc H and f matrices for quadprog
            % x0: current state
            % Xn = A*x0+B*U % prediction in horizon
            % dX = Xn-Xr
            % dX'*Q*dX = U'*B'*Q*B*U + 2(A*x0-Xr)'*Q*B*U + (x0'*x0 term)
            % dU = U - Ur
            % dU'*R*dU = U'*R*U - 2*Ur*R*U
            % dpU = U - Up
            % U'*Rp*U - 2*Up*Rp*U
            H = 2*(B'*Q*B+R+Rp);
            H = (H+H')/2;
            f = (2*(A*x0 - Xr)'*Q*B - 2*Ur'*R - 2*Up'*Rp)';

        end
        function result2input(obj)
            obj.result.pre_u = obj.input.u;
            obj.input.pre_u = obj.result.pre_u;
        end
        %% 目標軌道生成
        function [xr] = generate_reference(obj)
            xr = zeros(obj.param.total_size, obj.H);    % initialize
            RefTime = obj.self.reference.time_var.func; % 時間関数の取得
            for h = 0:obj.param.H-1
                t = obj.param.t + obj.param.dt * h; % reference生成の時刻をずらす
                ref = RefTime(t);
                xr(1:3, h+1) = ref(1:3);
                xr(7:9, h+1) = ref(5:7);
                xr(4:6, h+1) =   [0;0;0]; % 姿勢角
                xr(10:12, h+1) = [0;0;0];
                xr(13:16, h+1) = obj.param.ref_input; % MC -> 0.6597,   HL -> 0
            end
            % g = 9.81;
            % prev_euler = zeros(3,1);
            % for h = 0:obj.H-1
            %     t   = obj.param.t + obj.param.dt * h;    % reference生成の時刻をずらす
            %     ref = RefTime(t);                        % 20x1
            %     acc = ref(9:11);                         % ddx, ddy, ddz
            %     yaw = ref(4);                            % yaw
            %     s  = acc + [0;0;g];
            %     b3 = s / norm(s);
            %     b1c = [cos(yaw); sin(yaw); 0];
            %     v = cross(b3, b1c);
            %     if norm(v) < 1e-6
            %         if abs(b3(3))<0.9, b1=[0;0;1]; else, b1=[1;0;0]; end
            %         b2 = cross(b3,b1); b2=b2/norm(b2); b1=cross(b2,b3);
            %     else
            %         b2 = v/norm(v);  b1 = cross(b2,b3);
            %     end
            %     Rd = [b1,b2,b3];
            %     phi   = atan2(Rd(3,2), Rd(3,3));
            %     theta = asin(-Rd(3,1));
            %     psi   = atan2(Rd(2,1), Rd(1,1));
            %     euler = [phi;theta;psi];
            %     if h == 0 && obj.H > 1
            %         t_next   = t + obj.param.dt;
            %         ref_next = RefTime(t_next);
            %         acc_n    = ref_next(9:11);
            %         yaw_n    = ref_next(4);
            %         s_n  = acc_n + [0;0;g];
            %         b3_n = s_n / norm(s_n);
            %         b1c_n = [cos(yaw_n); sin(yaw_n); 0];
            %         v_n = cross(b3_n, b1c_n);
            %         if norm(v_n) < 1e-6
            %             if abs(b3_n(3))<0.9, b1_n=[0;0;1]; else, b1_n=[1;0;0]; end
            %             b2_n = cross(b3_n,b1_n); b2_n=b2_n/norm(b2_n); b1_n=cross(b2_n,b3_n);
            %         else
            %             b2_n = v_n/norm(v_n);  b1_n = cross(b2_n,b3_n);
            %         end
            %         Rd_n = [b1_n,b2_n,b3_n];
            %         phi_n   = atan2(Rd_n(3,2), Rd_n(3,3));
            %         theta_n = asin(-Rd_n(3,1));
            %         psi_n   = atan2(Rd_n(2,1), Rd_n(1,1));
            %         euler_n = [phi_n;theta_n;psi_n];
            %         euler_dot = (euler_n - euler) / obj.param.dt;
            %         euler_dot(3) = ref(8);
            %     elseif h == 0 && obj.H == 1
            %         euler_dot = [0;0;ref(8)];
            %     else
            %         euler_dot = (euler - prev_euler) / obj.param.dt;
            %         euler_dot(3) = ref(8);
            %     end
            %     prev_euler = euler;
            %     T = [ 1, 0, -sin(theta);
            %         0, cos(phi),  cos(theta)*sin(phi);
            %         0, -sin(phi), cos(theta)*cos(phi) ];
            %     w = T * euler_dot;
            %     xr(1:3,   h+1) = ref(1:3);
            %     xr(7:9,   h+1) = ref(5:7);
            %     xr(4:6,   h+1) = euler;
            %     xr(10:12, h+1) = w;
            %     xr(13:16, h+1) = obj.result.input(:,1);
            % end
        end

        function show(obj)
            % clc;
            % est_print = obj.self.estimator.result.state;
            est_print = obj.self.estimator.result.state;
            % fprintf("==================================================================\n")
            % fprintf("==================================================================\n")
            % fprintf("ps: %f %f %f",est_print.p(1), est_print.p(2), est_print.p(3));
            % fprintf("\n");
            fprintf("ps: %f %f %f \t vs: %f %f %f \t qs: %f %f %f \n",...
                est_print.p(1), est_print.p(2), est_print.p(3),...
                est_print.v(1), est_print.v(2), est_print.v(3),...
                est_print.q(1), est_print.q(2), est_print.q(3)); % s:state 現在状態
            % fprintf("pr: %f %f %f",obj.state.ref(1,1), obj.state.ref(2,1), obj.state.ref(3,1));
            fprintf("pr: %f %f %f \t vr: %f %f %f \t qr: %f %f %f \n", ...
                obj.state.ref(1,1), obj.state.ref(2,1), obj.state.ref(3,1),...
                obj.state.ref(7,1), obj.state.ref(8,1), obj.state.ref(9,1),...
                0, 0, obj.state.ref(6,1))                             % r:reference 目標状態
            fprintf("t: %f \t input: %f %f %f %f \t J: %f \t sigma: %f", ...
                obj.param.t, obj.result.input(1), obj.result.input(2), obj.result.input(3), obj.result.input(4), obj.result.bestcost(1),obj.input.sigma(1));
            fprintf("\n");
            fprintf("\n");
        end
    end
end
