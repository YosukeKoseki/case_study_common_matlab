%% Explanation
% 2025/06 Created by KOSEKI

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fsave = 1;
% [Recomendation] Initially, you should check the figure with fsave = 0, then chose save style.
% [推奨] 最初はfsave = 0でfigureを確認し，その後 保存形式を選択
% 0:no save
% 1:save as ".fig"
% 2:save as ".png"
% 3:save as ".pdf"

ftitle = 0; % defalt=1 -> with figure title

%%%%%%%%%%%%%%%%%%%%%%%% chose target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% settings.target = ["p", "q", "v", "w", "input", "p1-p2", "p1-p2-p3"];
settings.target = ["p", "p1-p2-p3"];
%                                                                   %
% p: position    q: angle    v: velocity    w: angular velocity     %
% input: controller input                                           %
% p1-p2: x-y 2D plot    p1-p2-p3: x-y-z 3D plot                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings.fontsize = 11;      % default=11
settings.linewidth = 0.5;    % default=0.5
settings.agent_id = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if logger.fExp == 1
    settings.attribute = ["e", "s", "r"];
else
    settings.attribute = ["e", "s", "r", "p"];
end

for i=1:length(settings.target)
    fcolor = 1;
    if settings.target(i)=="q"
        tmp = settings.attribute;
        tmp(3) = [];
    elseif settings.target(i)=="v"
        tmp = settings.attribute;
        tmp(2) = [];
    elseif settings.target(i)=="w"
        tmp = settings.attribute;
        tmp(2:3) = [];
    elseif settings.target(i)=="input"
        tmp = settings.attribute;
        tmp(2:3) = [];
    elseif settings.target(i)=="p1-p2" || settings.target(i)=="p1-p2-p3"
        tmp = settings.attribute;
        fcolor = 0;
    else
        tmp = settings.attribute;
    end
    att = select_attribute(settings.target(i), tmp);
    logger.plot({settings.agent_id, settings.target(i), att}, ...
        fig_num=i, color=fcolor, ...
        fontsize=settings.fontsize, linewidth=settings.linewidth)
    if ftitle == 0
        title([])
    end

    if fsave == 1
        savefig(['plot/fig/', char(settings.target(i)), '.fig']);
    elseif fsave == 2
        savefig(['plot/fig/', char(settings.target(i)), '.png']);
    elseif fsave == 3
        savefig(['plot/fig/', char(settings.target(i)), '.pdf']);
    end
end

%% function
function att = select_attribute(target, attribute)
text = cell(1, 4);
text{1} = ['<キーボードで[', char(target), ']用の属性を入力>\n'];%'\n<Keybord input attribute for [', char(target), ']>\n', 
text{2} = ['{', char(strjoin(attribute, "")), '}が使えます  '];%'You can use {', char(strjoin(attribute, "")), '}\n
if length(attribute) == 4
    text{3} = ['ex) ', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))],...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3)), char(attribute(4))],'\n'];
elseif length(attribute) == 3
    text{3} = ['ex) ', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))], '\n'];
elseif length(attribute) == 2
    text{3} = ['ex) ', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], '\n'];
else
    text{3} = ['ex) ', char(attribute(1)), '\n'];
end
text{4} = 'Input attribute: ';
fprintf([text{1:3}])
while true
    att = string(input([text{4}], 's'));
    chars = string(split(att, ""));
    chars(chars == "") = []; 
    if all(ismember(chars, attribute))
        break;
    else
        fprintf('!!!%s is incorrect!!! ', att)
    end
end
end
