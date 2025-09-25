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
    % % ４班　髙山幸汰　2312066
    % x = 0.5*cos(2*pi*t/T)+0.4 *cos(6*pi*t/T);
    % y = 0.5*sin(2*pi*t/T)-0.4* sin(6*pi*t/T);

    % % ４班　島野嵩士　2312047
    % x = sin(2*pi*t/T)/1.5-sin(pi*t/T)/10-sin(pi*t/T/2)/8;
    % y = cos(2*pi*t/T)/1.5-cos(pi*t/T)/10-cos(pi*t/T/2)/8;

    % % 4班　桶本海夢　2312025
    % x = 1*sin(2*2*pi/T*t);
    % y = 1*sin(3*2*pi/T*t);

    % 4班　宮崎優怜　2312122
    x = (0.3 + 0.1*cos(10*pi*t/T)).*cos(2*pi*t/T);
    y = (0.3 + 0.1*cos(10*pi*t/T)).*sin(2*pi*t/T);

    ref = @(t) [x_0+x;y_0+y;z_0;0];
end

