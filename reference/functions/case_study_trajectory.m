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
    % % 2班　遠藤翔　2312017
    % x = 0.6*cos(t/T)-0.1*cos(6*t/T);
    % y = 0.6*sin(t/T)-0.1*sin(6*t/T);

    % % 2班　平児玉裕翔　2312092
    % x = sin(3*2*pi*t/T)*cos(2*pi*t/T);
    % y = sin(3*2*pi*t/T)*sin(2*pi*t/T);

    % ２班　鈴木亮大　2312055
    x = cos(2*pi/T*t)*sin(pi/T*t);
    y = sin(2*pi/T*t);
    ref = @(t) [x_0+x;y_0+y;z_0;0];
end

