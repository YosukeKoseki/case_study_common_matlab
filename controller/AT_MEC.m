classdef AT_MEC
    %クアッドコプタ用階層型線形化ダイナミクスに対してAT-MECを適用
    % 既存の制御則との併用を想定
    % 作成者：小関　2025/9月
    
    properties
        self
        result
        param
        parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
    end
    
    methods
        function obj = AT_MEC(self)
            obj.self = self;
            obj.param = self.parameter.get(obj.parameter_name);
            obj.result.nominal_input = zeros(self.estimator.model.dim(2),1);
            obj.result.delta_input = zeros(self.estimator.model.dim(2),1);
            obj.result.input = zeros(self.estimator.model.dim(2),1);
            obj.x_pre = self.estimator.result.state.get;
            obj.pre_input = zeros(self.estimator.model.dim(2),1);
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 このメソッドの概要をここに記述
            %   詳細説明をここに記述
            outputArg = obj.Property1 + inputArg;
        end
    end
end

