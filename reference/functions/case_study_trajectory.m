function ref = case_study_trajectory(param)
%TMP_CASE_STUDY_TRAGECTORY 2025年度事例研究 ドローン全体実験用関数　作成者：小関
%   媒介変数表示された，時変な関数をリファレンスとして設定する
%   [Default prameters]
%       pram.freq = 10 % 周期
%       param.init = [0 0 1] % 開始点

arguments
    param.freq = 10 % 周期
    param.init = [0 0 1] % 開始点
end
    %% 初期化
    syms t real
    T = param.freq;
    x_0 = param.init(1);
    y_0 = param.init(2);
    z_0 = param.init(3);
    
    %% 動作チェック用
    % URL:https://manabitimes.jp/math/898 より
    % % 円軌道
    % x = x_0 + sin(2*pi*t/T);
    % y = y_0 + cos(2*pi*t/T);
    % z = z_0;

    % アステロイド曲線（星芒形）
    x = x_0 + (sin(2*pi*t/T))^3;
    y = y_0 + (cos(2*pi*t/T))^3;
    z = z_0;

    % % カージオイド曲線（心臓形）
    % x = x_0 + 0.5*(1+cos(2*pi*t/T))*cos(2*pi*t/T);
    % y = y_0 + 0.5*(1+cos(2*pi*t/T))*sin(2*pi*t/T);
    % z = z_0;

    % % リサージュ図形
    % A = 1; % 振幅
    % B = 1; % 振幅
    % a = 3; % 角周波数
    % b = 4; % 角周波数
    % delta = 0; % 位相差[rad]
    % x = x_0 + A*sin(a*2*pi*t/T + delta);
    % y = y_0 + B*sin(b*2*pi*t/T);
    % z = z_0;

    %% 持ってきてもらったリファレンス情報を追加
    
    ref = @(t) [x;y;z;0];
end

