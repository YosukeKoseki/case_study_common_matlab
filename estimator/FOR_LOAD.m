classdef FOR_LOAD < handle
      properties
        result                  % ���ݎ����̌���
        rigid_num               % ���̔ԍ�
        self                    % agent
        tt0                     % takeoff�̊J�n����
        tl0                     % landing�̊J�n����
        tte         = 10        % �Z���T�[�l�����b��100%�g����
        tle         = 5         % �Z���T�[�l�����b��0%�g����
        ratet                   % �񎟊֐���0-1�̊Ԃŕω����邽�߂̒萔
        ratel                   % �񎟊֐���0-1�̊Ԃŕω����邽�߂̒萔
        inispL                  % �����̌������ʒu
        isAir       = 0         % �@�̂���荂�x��荂���Ȃ��Ă���
        isGround    = 0         % �n�ʔ���̏�����
        cableL_landing          % landing�J�n���̋@�̂ƌ������̋���
        k=0
    end
    
    methods
        function obj = FOR_LOAD(self,varargin)
            %�������l�����Ƃ��ɏ�������͐���l���g�����ق��������ꍇ�����邩��
            obj.self                = self;
            if ~isempty(varargin)
                if isfield(varargin{1},'rigid_num')
                    obj.rigid_num   = varargin{1,1}.rigid_num;
                end
            end
            obj.result.state        = STATE_CLASS(struct('state_list',["p","q","pL","pT","real_pL","isGround","k"],"num_list",[3,4,3,3,1,1]));
            obj.ratet               = 1/obj.tte^2;%�񎟊֐���0-1�̊Ԃŕω�����
            obj.ratel               = 1/obj.tle^2;%�񎟊֐���0-1�̊Ԃŕω�����
        end
        
        function result = do(obj,varargin)
            %   param : optional
            tc          = varargin{1}.t;                                     % ���ݎ���
            cha         = varargin{2};                                       % phase
            sp          = obj.self.sensor.motive.result.state.p;                % �@�̈ʒu
            sq          = obj.self.sensor.motive.result.state.q;                % �@�̊p�x
            spL         = obj.self.sensor.motive.result.rigid(obj.rigid_num).p; % �������ʒu
            ipL         = sp -[0;0;obj.self.parameter.get("cableL")];           % �@�̂̐^���ɂ���Ɖ��肵���Ƃ��̌������ʒu
            
            %flight
            if strcmp(cha,'f')
                obj.tt0         =[];                                            % takeoff�̊J�n���Ԃ�������
                obj.tl0         =[];                                            % landing�̊J�n���Ԃ�������
                obj.isGround    =0;                                             % �n�ʔ���̏�����
            %take off
            elseif strcmp(cha,'t')
                if isempty(obj.inispL)
                    obj.inispL  = spL;                                                  % �����̌������ʒu
                end
                % �������̏�������+�@�̂̑S����荂���Ȃ�����Z���T�l���g���n�߂�
                % if sp(3)> obj.inispL(3) + 0.3 || obj.isAir
                if sp(3)> obj.inispL(3) + obj.self.parameter.get("cableL")*0.9 || obj.isAir
                    obj.isAir   = 1;
                    if isempty(obj.tt0)
                        obj.tt0 = tc;
                    end
                    t           = min((tc - obj.tt0),obj.tte);                   %takeoff�̌o�ߎ��Ԃ��Z���T�l�g�p��100%�ɂȂ鎞�Ԃ��z���Ȃ��悤�ɂ���
                    obj.k           = obj.ratet*t^2;                                 %�Z���T�l���f����
                    spL(1:2)    = sp(1:2) + obj.k*(spL(1:2) - sp(1:2));              %�������ʒu�Ƌ@�̈ʒu�̍��ɔ��f�����������ăZ���T�l�𔽉f
                    spL(3)      = ipL(3);
                    obj.k
                %臒l���z���Ȃ�������@�̂̐^���Ɍ����������邱�Ƃɂ���
                else
                    spL         = ipL;
                end
            %landing
            elseif strcmp(cha,'l') 
                %�n�ʂɂ������̔���
                if ~obj.isGround 
                    % ���������ʐ�����s���Ă���ꍇ
                    if isprop(obj.self.estimator.result.state,"mL")
                        %�R���g���[���N���X�Œn�ʂɂ������肪���邩
                        if obj.self.controller.isGround
                            obj.isGround        = 1;
                        end
                    % ���������ʐ��肵�Ă��Ȃ��ꍇ��landing�J�n���̋@�̂ƌ������̋�������̍��ɂ���Ĕ��f
                    else
                        if isempty(obj.cableL_landing)
                            obj.cableL_landing  = sp - spL;                     % landing�J�n���̋@�̂ƌ������̋���
                        end
                        % landing�J�n���̋@�̂ƌ������̋�����z�����̔����̒�����茻�݂̍��̋����̕����Z���ꍇ
                        if sp(3) - spL(3) < obj.cableL_landing(3)*0.9
                            obj.isGround        = 1;
                        end
                    end
                end
                sfG = obj.isGround
                % �n�ʂɂ�������ɂȂ�����Z���T�l���g���n�߂�
                % if obj.isGround
                    if isempty(obj.tl0)
                        obj.tl0 = tc;
                    end
                    t           = min(tc - obj.tl0, obj.tle);                   % landing�̌o�ߎ��Ԃ��Z���T�l�g�p��0%�ɂȂ鎞�Ԃ��z���Ȃ��悤�ɂ���
                    obj.k           = -obj.ratel*t^2 + 1;                           % �Z���T�l���f����
                    spL(1:2)    = sp(1:2) + obj.k*(spL(1:2) - sp(1:2));             % �������ʒu�Ƌ@�̈ʒu�̍��ɔ��f�����������ăZ���T�l�𔽉f
                    % spL    = ipL + obj.k*(ipL - spL);             % �������ʒu�Ƌ@�̈ʒu�̍��ɔ��f�����������ăZ���T�l�𔽉f
                % end
                spL(3)      = ipL(3);
            else
                spL             = ipL;                                          % �@�̂̐^���ɂ���Ɖ��肵���Ƃ��̌������ʒu
                obj.tt0         = [];                                           % takeoff�̊J�n���Ԃ�������
                obj.tl0         = [];                                           % landing�̊J�n���Ԃ�������
                obj.isGround    = 0;                                            % �n�ʔ���̏�����
            end
        %log
            obj.result.state.p          = sp;
            obj.result.state.q          = sq;
            obj.result.state.pL         = spL;
            obj.result.state.real_pL    = obj.self.sensor.motive.result.rigid(obj.rigid_num).p;
            obj.result.state.pT         = (spL-sp)/norm(spL-sp);                % �@�̂���݂��������܂ł̒P�ʕ����x�N�g��
            obj.result.state.isGround         = obj.isGround;
            obj.result.state.k         = obj.k;
            result                      = obj.result;
        end
        function show()
        end
    end
end

