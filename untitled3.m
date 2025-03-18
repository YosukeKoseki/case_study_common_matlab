T= timer('Period', 0.5,... % poling interval
            'ExecutionMode', 'fixedSpacing', ... % execution mode
            'TasksToExecute', Inf, ... % trial number
            'TimerFcn', @fnc,...
            'ErrorFcn',@efn,...
            'StopFcn',@sfn);
k = 1;

tic
inv(rand(3000));
toc
%%
start(T)
while 1
    pause(0.4);
    disp(k)
    if rem(k,10) == 0
        inv(rand(3000));
        % x  = input("ccc");
        % pause(1);
        % % keyboard
        % if x == 1
        %     error("hoge");
        % end
    end
    k= k+1;
end
function fnc(a,b)
    disp(b.Data)
end
function efn(a,b)
    disp("eeeeeerrrrrrrr")
    % stop(a);
    % delete(a);
        tt=timerfind;
disp(tt)
for i = 1:length(tt)
stop(tt(i))
delete(tt(i))
end
end
function sfn(a,b)
    disp("stooooop")
    stop(a);
    delete(a);
    tt=timerfind;
disp(tt)
for i = 1:length(tt)
stop(tt(i))
delete(tt(i))
end
end
%%
%  tt=timerfind;
% disp(tt)
% for i = 1:length(tt)
% stop(tt(i))
% delete(tt(i))
% end

stop(timerfind)
delete(timerfind)