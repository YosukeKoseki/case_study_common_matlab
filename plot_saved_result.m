%% 初期化&パスの設定
clc; clear;
tmp = matlab.desktop.editor.getActive;
cd(strcat(fileparts(tmp.Filename), '../../../'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);

clear all
clc
%% データの読み込み
fprintf('MATファイルを選択してください:')
if ~exist('filename', 'var')
    [filename, pathname] = uigetfile('*.mat', 'MATファイルを選択してください');
end
fprintf(filename);
fullpath = fullfile(pathname, filename);
logger = LOGGER(fullpath);

%% プロット
fsave = 0; % 保存用フラグ
% 0:no save
% 1:save as ".fig"
% 2:".png"
% 3:".pdf"

for agent_id = 1:logger.target
end
% logger.data(1, "p", "e", ranget=[0 50])
logger.plot({agent_id, "p", "er"}, fig_num=1, fontsize=18, linewidth=2)
logger.plot({agent_id, "input", ""}, 'fig_num',2)
% logger.plot({1, "v", "er"})

% time = Log.log.Data.t;
% logger = LOGGER(1, size(1:find(time == max(time)), 2), Log.log.fexp, [], []);
% logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);