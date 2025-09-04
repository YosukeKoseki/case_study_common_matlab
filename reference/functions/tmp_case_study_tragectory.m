function ref = tmp_case_study_tragectory(param)
%TMP_CASE_STUDY_TRAGECTORY 2025年度事例研究 ドローン全体実験用関数
%   媒介変数表示された，時変な関数をリファレンスとして設定する

arguments
    param.freq = 10 % 周期
    param.init = [0 0 0] % 開始点
end

syms t real
T = param.freq;

% 円軌道
x = sin(2*pi*t/T);
y = cos(2*pi*t/T);
z = 1;

ref = @(t) [x;y;z;0];
end

