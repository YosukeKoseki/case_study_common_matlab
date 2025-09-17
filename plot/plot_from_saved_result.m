%% 説明
% 2025/06 作成者：小関
% Exp / Simデータをプロットすることができるファイル
% 最初は全てのセクションを実行する．
% データの読み込みができたら，プロットセクションだけ実行すれば手間が省ける．
% プロットセクションのsettingsを変更して調整することでmainGUIの画面上とは違ういい感じのグラフが取れる
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 50行付近のsettingsは色々見てください！！  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 分からないことや追加したい機能などあったらSlackで聞いてください
% 凡例に関しては，書き方・位置を要検討
% プロットしたいフェーズを選べるようになると嬉しい

%% 初期化&パスの設定
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(erase(tmp.Filename, "plot\plot_from_saved_result.m")));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');

%% データの読み込み
fprintf('MATファイルを選択してください:')
[filename, pathname] = uigetfile('*.mat', 'MATファイルを選択してください');
fprintf(filename);
fullpath = fullfile(pathname, filename);
logger = LOGGER(fullpath);

%% プロット
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fsave = 0;
% [Recomendation] Initially, you should check the figure with fsave = 0, then chose save style.
% [推奨] 最初はfsave = 0でfigureを確認し，その後 保存形式を選択
% 0:no save
% 1:save as ".fig"
% 2:save as ".png"
% 3:save as ".jpg"
% 4:save as ".pdf"
% 5:save as ".eps"

ftitle = 0; % defalt=1 -> グラフタイトルあり
settings.fcolor = 0; % default=1 -> フェーズごとの背景色あり

%%%%%%%%%%%%%%%%%%%%%%%% chose target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
settings.target = ["p", "q", "v", "w", "input", "p1-p2", "p1-p2-p3"];
% settings.target = ["input", "inner_input1:4"];
% プロットしたいグラフの情報                                        %
% p: position    q: angle    v: velocity    w: angular velocity     %
% input: controller input    inner_input1:4: transmitter input      %
% p1-p2: x-y 2D plot    p1-p2-p3: x-y-z 3D plot                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% settings.phase = "atfl";
settings.phase = "f";
settings.fontsize = 18;    % default=11 オススメ=18
settings.linewidth = 1.5;    % default=0.5 オススメ=1.5
settings.agent_id = 1;

settings.savefolder = 'plot/fig/';  % default
% settings.savename = '';

% estimator, sensor, reference, (plant) どの値を表示するかは
% 途中のキーボード入力で決定します．
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if logger.fExp == 1
    settings.attribute = ["e", "s", "r"];
else
    settings.attribute = ["e", "s", "r", "p"];
end

for i=1:length(settings.target)
    fcolor = settings.fcolor;
    switch settings.target(i)
        case "p"
            ylabel = "Position [m]";
            tmp = settings.attribute;
            att = select_attribute(settings.target(i), tmp);
        case "q"
            ylabel = "Attitude [rad]";
            tmp = settings.attribute;
            tmp(3) = []; % "esr"の内，無いものを消去
            att = select_attribute(settings.target(i), tmp);
        case "v"
            ylabel = "Velocity [m/s]";
            tmp = settings.attribute;
            tmp(2) = [];
            att = select_attribute(settings.target(i), tmp);
        case "w"
            ylabel = "Angular velocity [rad/s]";
            tmp = settings.attribute;
            tmp(2:3) = [];
            att = select_attribute(settings.target(i), tmp);
        case "input"
            ylabel = "Controller input [N],[Nm]";
            tmp = settings.attribute;
            tmp(2:3) = [];
            att = "";
        case "inner_input"
            ylabel = "Transmitter input [N],[Nm]";
            tmp = settings.attribute;
            tmp(:) = [];
            tmp = "";
            att = "";
        case "inner_input1:4"
            ylabel = "Transmitter input [Nm], [N]";
            tmp = settings.attribute;
            tmp(:) = [];
            tmp = "";
            att = "";
        case "p1-p2"
            xlabel = "$x$ [m]";
            ylabel = "$y$ [m]";
            tmp = settings.attribute;
            fcolor = 0;
            att = select_attribute(settings.target(i), tmp);
        case "p1-p2-p3"
            xlabel = "$x$ [m]";
            ylabel = "$y$ [m]";
            zlabel = "$z$ [m]";
            tmp = settings.attribute;
            fcolor = 0;
            att = select_attribute(settings.target(i), tmp);
    end
    logger.plot({settings.agent_id, settings.target(i), att}, ...
        'fig_num',i, 'color',fcolor, "phase",settings.phase, ...
        'FontSize',settings.fontsize, 'Linewidth',settings.linewidth)

    fig = gcf;
    ax = gca;
    
    chars = string(split(att, ""));
    chars(chars == "") = [];
    switch settings.target(i)
        case "p1-p2"
            set(ax.XLabel, 'String', xlabel, 'Interpreter','latex')
            set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
            data = logger.data(1,"p","e","phase",settings.phase);
            xlim([min(data(:,1)) max(data(:,1))])
            ylim([min(data(:,2)) max(data(:,2))])
        case "p1-p2-p3"
            set(ax.XLabel, 'String', xlabel, 'Interpreter','latex')
            set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
            set(ax.ZLabel, 'String', zlabel, 'Interpreter','latex')
            data = logger.data(1,"p","e","phase",settings.phase);
            xlim([min(data(:,1)) max(data(:,1))])
            ylim([min(data(:,2)) max(data(:,2))])
            zlim([min(data(:,3)) max(data(:,3))])
        case "inner_input1:4"
            set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
            legend = set_legend(settings.target(i), chars);
            set(ax.Legend, 'String', legend, 'Interpreter','latex');
            data = logger.data(1,settings.target(i),"","phase",settings.phase);
            y_min=0;
            y_max=0;
            for j=1:size(data,2)
                if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                if y_max<max(data(:,j)), y_max=max(data(:,j)); end
            end
            ylim([y_min y_max])
        otherwise
            set(ax.YLabel, 'String', ylabel, 'Interpreter','latex')
            legend = set_legend(settings.target(i), chars);
            set(ax.Legend, 'String', legend, 'Interpreter','latex');
            data = logger.data(1,settings.target(i),"e","phase",settings.phase);
            y_min=0;
            y_max=0;
            for j=1:size(data,2)
                if y_min>min(data(:,j)), y_min=min(data(:,j)); end
                if y_max<max(data(:,j)), y_max=max(data(:,j)); end
            end
            ylim([y_min y_max])
    end
    if ftitle == 0
        set(ax.Title, 'String', [])
    end
    set(ax.Legend, 'Location','southeast', 'FontSize',settings.fontsize-4);

    if ~exist('plot/fig', 'dir')
        mkdir('plot/fig')
    end
    if settings.savename==1
        if fsave == 1, savefig([settings.savefolder, erase(filename, '.mat'), '_', erase(char(settings.target(i)),':'), '.fig']);
        elseif fsave == 2, saveas(fig, [settings.savefolder, erase(filename, '.mat'), '_', erase(char(settings.target(i)),':'), '.png']);
        elseif fsave == 3, saveas(fig, [settings.savefolder, erase(filename, '.mat'), '_', erase(char(settings.target(i)),':'), '.jpg']);
        elseif fsave == 4, saveas(fig, [settings.savefolder, erase(filename, '.mat'), '_', erase(char(settings.target(i)),':'), '.pdf']);
        elseif fsave == 5, saveas(fig, [settings.savefolder, erase(filename, '.mat'), '_', erase(char(settings.target(i)),':'), '.eps'],'epsc2');
        end
    else
        if fsave == 1, savefig([settings.savefolder, settings.savename, '_', erase(char(settings.target(i)),':'), '.fig']);
        elseif fsave == 2, saveas(fig, [settings.savefolder, settings.savename, '_', erase(char(settings.target(i)),':'), '.png']);
        elseif fsave == 3, saveas(fig, [settings.savefolder, settings.savename, '_', erase(char(settings.target(i)),':'), '.jpg']);
        elseif fsave == 4, saveas(fig, [settings.savefolder, settings.savename, '_', erase(char(settings.target(i)),':'), '.pdf']);
        elseif fsave == 5, saveas(fig, [settings.savefolder, settings.savename, '_', erase(char(settings.target(i)),':'), '.eps'],'epsc2');
        end
    end
end
disp_rmse(logger,settings.phase)

% % functions
function att = select_attribute(target, attribute)
text = cell(1, 4);
text{1} = ['\n<キーボードで「', char(target), '」用の値の種類を入力>\n'];%'\n<Keybord input attribute for [', char(target), ']>\n', 
text{2} = ['   {', char(strjoin(attribute, "")), '}が使えます  '];%'You can use {', char(strjoin(attribute, "")), '}\n
if length(attribute) == 4
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))],...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3)), char(attribute(4))],'\n'];
elseif length(attribute) == 3
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], ...
        ', ', [char(attribute(1)), char(attribute(2)), char(attribute(3))], '\n'];
elseif length(attribute) == 2
    text{3} = ['例）', char(attribute(1)), ', ', [char(attribute(1)), char(attribute(2))], '\n'];
else
    text{3} = ['例）', char(attribute(1)), '\n'];
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



function legend = set_legend(target, chars)
att_map = containers.Map({'e', 's', 'r', 'p'},...
    {'est.', 'sen.', 'ref.', 'pla.'});
legend_num = numel(chars) * 3;
legend = cell(1, legend_num);
switch target
    case "p"
        for i=1:numel(chars)
            legend{3*i-2} = "$x$ " + att_map(chars(i));
            legend{3*i-1} = "$y$ " + att_map(chars(i));
            legend{3*i} = "$z$ " + att_map(chars(i));
        end
    case "v"
        for i=1:numel(chars)
            legend{3*i-2} = "$v_x$ " + att_map(chars(i));
            legend{3*i-1} = "$v_y$ " + att_map(chars(i));
            legend{3*i} = "$v_z$ " + att_map(chars(i));
        end
    case "q"
        for i=1:numel(chars)
            legend{3*i-2} = "$\theta_{roll}$ " + att_map(chars(i));
            legend{3*i-1} = "$\theta_{pitch}$ " + att_map(chars(i));
            legend{3*i} = "$\theta_{yaw}$ " + att_map(chars(i));
        end
    case "w"
        for i=1:numel(chars)
            legend{3*i-2} = "$\omega_{roll}$ " + att_map(chars(i));
            legend{3*i-1} = "$\omega_{pitch}$ " + att_map(chars(i));
            legend{3*i} = "$\omega_{yaw}$ " + att_map(chars(i));
        end
    case "input"
        legend{1} = "$u_{thrust}$";
        legend{2} = "$u_{roll}$";
        legend{3} = "$u_{pitch}$";
        legend{4} = "$u_{yaw}$";
    case "inner_input1:4"
        legend{1} = "$u_{roll}$";
        legend{2} = "$u_{pitch}$";
        legend{3} = "$u_{thrust}$";
        legend{4} = "$u_{yaw}$";
end
end


function disp_rmse(logger, phase)
target = ["p","v"];
for i=1:length(target)
    ref = logger.data(1,target(i),"r", "phase",phase);
    data = logger.data(1,target(i),"e", "phase",phase);
    RMSE = rmse(ref, data, 1);
    fprintf('%s RMSE:\n', target(i))
    disp(RMSE)
    disp(sum(RMSE))
end
end
