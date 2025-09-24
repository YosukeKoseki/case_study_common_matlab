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
    % % % URL:https://manabitimes.jp/math/898 より
    % 円軌道
    x = sin(2*pi*t/T);
    y = cos(2*pi*t/T);

    % % アステロイド曲線（星芒形）
    % x = (sin(2*pi*t/T))^3;
    % y = (cos(2*pi*t/T))^3;

    % % カージオイド曲線（心臓形）
    % x = 0.5*(1+cos(2*pi*t/T))*cos(2*pi*t/T);
    % y = 0.5*(1+cos(2*pi*t/T))*sin(2*pi*t/T);

    % % リサージュ図形
    % A = 1; % 振幅
    % B = 1; % 振幅
    % a = 3; % 角周波数
    % b = 4; % 角周波数
    % delta = 0; % 位相差[rad]
    % x = A*sin(a*2*pi*t/T + delta);
    % y = B*sin(b*2*pi*t/T);

    %% 持ってきてもらったリファレンス情報を追加
    % % % ３班　佐藤里咲　2312044
    % x = 16 * (sin(t)).^3;
    % y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
    
    % % ３班　松坂光　2312096
    % x = sin(1*pi*t/T)*cos(2*pi*t/T);
    % y = sin(1*pi*t/T)*sin(2*pi*t/T);
    
    % % ３班　荻原滉明　2312024
    % x = sin(2*pi*t/T);
    % y = 0.5*sin(4*pi*t/T);

    % ３班　関野凌雅　2312059
    theta = (2*pi*0.5 / T) * t;
    x = 0.035 * exp(b * theta) .* cos(theta);
    y = 0.105 * exp(b * theta) .* sin(theta);

    ref = @(t) [x_0+x;y_0+y;z_0;0];
end

