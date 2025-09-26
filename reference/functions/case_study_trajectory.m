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
    % % 円軌道
    % x = sin(2*pi*t/T);
    % y = cos(2*pi*t/T);

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
    % 1班　山田遥斗　2312114
    x = cos(2*pi*t/T);
    y = sin(4*pi*t/T);

    % % 1班　泉崎優斗　2312007
    % x = sin(2*pi*2*t/T);
    % y = cos(2*pi*3*t/T);

    % % 1班　竹内優音　2312067
    % x = 0.5*(cos(t)*(1+cos(t)));
    % y = 0.5*(sin(t)*(1+cos(t)));

    % % 1班　鈴木颯太　2312054
    % x = cos(2 * 2*pi*t/10) .* cos(2*pi*t/10);
    % y = cos(2 * 2*pi*t/10) .* sin(2*pi*t/10);

    % % 1班 角谷航 2312027
    % T = 5; % 周期T=10だと長すぎるかも
    % x= (2/3)*cos(t/T)+(1/3)*cos(2*t/T);
    % y=(2/3)*sin(t/T)-(1/3)*sin(2*t/T);

    ref = @(t) [x_0+x;y_0+y;z_0;0];
end

