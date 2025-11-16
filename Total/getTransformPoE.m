function T = getTransformPoE(robot, q, bodyName, baseName)
%GETTRANSFORMPOE  用指数积（PoE）方法计算刚体间位姿
% GETTRANSFORMPOE  Compute relative pose between rigid bodies using PoE
%
%   T = getTransformPoE(robot, q, bodyName, baseName)
%
% 输入 (Inputs):
%   robot    : rigidBodyTree（建议 DataFormat = 'struct'）
%              rigidBodyTree object (recommended DataFormat = 'struct')
%
%   q        : 关节配置（struct 数组，homeConfiguration(robot) 那种）
%              Joint configuration as struct array (e.g. homeConfiguration(robot))
%
%   bodyName : 目标刚体名，例如 'panda_hand'
%              Name of target body, e.g. 'panda_hand'
%
%   baseName : 参考刚体名，例如 robot.BaseName
%              Name of reference body, e.g. robot.BaseName
%
% 输出 (Output):
%   T        : 4×4 齐次变换矩阵，在配置 q 下，从 baseName 到 bodyName：
%              ˆ{baseName}T_{bodyName}
%              4×4 homogeneous transform ^{baseName}T_{bodyName} at config q
%
% 调用风格与 Robotics System Toolbox 的 getTransform 一致，只是内部用 PoE：
% The calling style matches Robotics System Toolbox getTransform,
% but the internal computation uses the Product of Exponentials (PoE) method.

    % ----------- 输入预处理 / Input preprocessing -----------------------
    if nargin < 4
        % 若未指定 baseName，则默认使用刚体树根节点
        % If baseName is not provided, use the tree base as reference
        baseName = robot.BaseName;
    end

    if ~isstruct(q)
        error('目前 getTransformPoE 仅支持 DataFormat="struct" 的配置 q。 / Only DataFormat="struct" is supported.');
    end

    bodyName = char(bodyName);
    baseName = char(baseName);

    % ----------- 特殊情况：同一刚体 / Special case: same body ----------
    % bodyName 和 baseName 相同：返回单位阵
    % If bodyName equals baseName, the relative transform is identity
    if strcmp(bodyName, baseName)
        T = eye(4);
        return;
    end

    % ----------- 在“树根坐标系”下分别计算两个刚体的位姿 ----------------
    % Compute poses of baseName and bodyName w.r.t. the tree base frame
    rootName = robot.BaseName;  % 树根（通常当作空间坐标系） / Root frame (space frame)

    % baseName 等于 rootName 时，root->base 变换为单位阵
    % If baseName is the root, ^rootT_base is identity
    if strcmp(baseName, rootName)
        T_root_base = eye(4);
    else
        % 使用 PoE 计算 ^rootT_base
        % Compute ^rootT_base using PoE
        T_root_base = worldTransformPoE(robot, q, baseName);
    end

    % 使用 PoE 计算 ^rootT_body
    % Compute ^rootT_body using PoE
    T_root_body = worldTransformPoE(robot, q, bodyName);

    % ----------- 组合得到 baseName->bodyName 的变换 ----------------------
    % Compose transform from baseName to bodyName:
    % ^{base}T_{body} = (^{root}T_{base})^{-1} * ^{root}T_{body}
    T = T_root_base \ T_root_body;   % 等价于 inv(T_root_base)*T_root_body
end


%% ====== 内部函数：计算“树根 -> 指定刚体”的 PoE 变换 ====================
% Helper: compute ^{BaseName}T_{eeName} via PoE
function T = worldTransformPoE(robot, q, eeName)
% 在关节配置 q 下，使用 PoE 计算:
%   T = ˆ{BaseName}T_{eeName}
% 其中 BaseName = robot.BaseName
%
% Compute the transform from the tree base frame to eeName using PoE:
%   T = ^{BaseName}T_{eeName}
% with BaseName = robot.BaseName

    rootName = robot.BaseName;

    % 如果目标就是根节点，返回单位阵
    % If eeName is the root, return identity
    if strcmp(eeName, rootName)
        T = eye(4);
        return;
    end

    % 1) 找出从 root 到 eeName 的刚体链（可能包含 fixed 关节）
    % 1) Get body chain from root to eeName (may include fixed joints)
    [bodyChain, ~] = getBodyJointChain(robot, eeName);

    % 2) 在零位下构造该链条的 Slist 和 M
    %    只对 revolute / prismatic 关节生成螺旋向量，
    %    fixed 关节只累积到 T_home 中
    %
    % 2) Build Slist and M at zero configuration:
    %    - Only revolute/prismatic joints contribute twists
    %    - Fixed joints only modify T_home (constant transform)
    Slist      = [];       % 6×ndof，列数为非固定关节数 / 6×ndof, columns for non-fixed joints
    jointNames = {};       % 记录有自由度的关节名 / Names of non-fixed joints
    T_home     = eye(4);   % ^rootT_body_i(q=0)，从根到当前 body 的零位姿态
                           % ^rootT_body_i(q=0), base-to-body_i at zero config

    for i = 1:numel(bodyChain)
        bname = bodyChain{i};
        body  = getBody(robot, bname);

        % 父坐标系 -> 当前刚体 的固定变换（零位）
        % Fixed transform from parent frame to current body at zero config
        A_i = body.Joint.JointToParentTransform;

        % 累积得到 ^rootT_body_i
        % Accumulate: ^rootT_body_i = ^rootT_body_{i-1} * A_i
        T_home = T_home * A_i;

        % fixed 关节：不生成 twist，只更新 T_home
        % For fixed joints: no twist, only update T_home
        if strcmp(body.Joint.Type, 'fixed')
            continue;
        end

        % 提取当前关节原点和姿态（在 root 坐标系中表示）
        % Extract joint origin and orientation in root frame
        R = T_home(1:3, 1:3);
        p = T_home(1:3, 4);

        % 关节轴在“关节自身坐标系”下的表达
        % Joint axis expressed in its local frame
        axis_local = body.Joint.JointAxis(:);  % 3×1

        % 根据关节类型构造空间螺旋向量 S_i = [w; v]
        % Build spatial twist S_i = [w; v] according to joint type
        if strcmp(body.Joint.Type, 'revolute')
            omega = R * axis_local;   % ω 在 root 坐标系下的方向 / ω in root frame
            v     = -cross(omega, p); % v = -ω × q
        elseif strcmp(body.Joint.Type, 'prismatic')
            omega = [0; 0; 0];        % 移动关节无旋转部分 / No rotational part
            v     = R * axis_local;   % v 为平移方向（在 root 坐标系下）/ v is translation dir
        else
            % 理论上不会到这里，其他类型视为无自由度
            % Should not reach here; treat others as no DoF
            omega = [0; 0; 0];
            v     = [0; 0; 0];
        end

        % 将当前关节的 twist 追加到 Slist
        % Append current joint twist to Slist
        Slist      = [Slist, [omega; v]];       % 6×(ndof)
        jointNames = [jointNames, {body.Joint.Name}];
    end

    % PoE 公式中的 M：零位时，root -> eeName 的位姿
    % M in PoE: pose ^rootT_ee at zero configuration
    M = T_home;

    % 3) 从 q 中提取对应非 fixed 关节的 thetalist
    % 3) Extract thetalist for non-fixed joints from configuration q
    ndof      = numel(jointNames);
    thetalist = zeros(ndof, 1);
    qNames    = {q.JointName};

    for i = 1:ndof
        jname = jointNames{i};
        idx = find(strcmp(qNames, jname), 1);
        if isempty(idx)
            error('给定配置 q 中不存在关节 "%s"。 / Joint "%s" not found in q.', jname, jname);
        end
        thetalist(i) = q(idx).JointPosition;
    end

    % 4) 使用 PoE 公式 T(θ) = e^[S1]θ1 ... e^[S_ndof]θ_ndof M
    % 4) Apply PoE: T(θ) = e^[S1]θ1 ... e^[S_ndof]θ_ndof M
    T = eye(4);
    for i = 1:ndof
        se3mat = vecToSe3(Slist(:, i));
        T = T * expm(se3mat * thetalist(i));
    end
    T = T * M;
end


%% ====== 内部：沿树从 base 走到 ee 的刚体/关节链 =======================
% Helper: find body/joint chain from BaseName to eeName
function [bodyChain, jointChain] = getBodyJointChain(robot, eeName)
% 从 BaseName 走到 eeName 的 body/joint 链（按 base -> ee 顺序）
% Get body/joint chain from BaseName to eeName (ordered base -> ee)

    bodyChain  = {};
    jointChain = {};

    % 起点：末端刚体（既支持名字，也支持 rigidBody 对象）
    % Start from the end body (supports both name and rigidBody object)
    if isa(eeName, 'rigidBody')
        body = eeName;
    else
        body = getBody(robot, eeName);
    end

    % 从 ee 沿父节点向上，直到 BaseName
    % Traverse parents from ee up to BaseName
    while ~strcmp(body.Name, robot.BaseName)
        % 当前 body 在链上
        % Current body is on the chain
        bodyChain{end+1}  = body.Name;
        jointChain{end+1} = body.Joint.Name;

        % 父节点可能是 rigidBody，也可能是名字
        % Parent may be a rigidBody or a name
        parent = body.Parent;

        % -------- 处理父节点（关键：Base 使用 robot.Base） -----------
        % Handle parent; special handling for robot.Base
        if isa(parent, 'rigidBody')
            % Parent 已经是 rigidBody 对象
            % Parent is already a rigidBody object
            if strcmp(parent.Name, robot.BaseName)
                body = robot.Base;   % 使用 Base 属性 / Use Base property
            else
                body = parent;
            end
        else
            % parent 是字符串名字
            % Parent is a name string
            parentName = char(parent);
            if strcmp(parentName, robot.BaseName)
                body = robot.Base;   % Base 用 robot.Base 取代 getBody
            else
                body = getBody(robot, parentName);
            end
        end
    end

    % 此时 body.Name == BaseName，链条收集完毕
    % At this point body.Name == BaseName, chain collection finished
    % 我们只保留带关节的刚体，不包含 Base 本身
    % Only keep bodies with joints, exclude the Base itself
    bodyChain  = fliplr(bodyChain);
    jointChain = fliplr(jointChain);
end


%% ====== 内部：twist 向量 → se(3) 矩阵 ================================
% Helper: convert twist (6×1) to se(3) matrix (4×4)
function se3mat = vecToSe3(S)
% VECTOSE3  将 6×1 螺旋向量 [w; v] 转为 4×4 se(3) 矩阵
% VECTOSE3  Convert 6×1 twist [w; v] into a 4×4 se(3) matrix

    omega = S(1:3);   % 角速度 / angular part
    v     = S(4:6);   % 线速度 / linear part

    % 反对称矩阵 [ω]_×
    % Skew-symmetric matrix [ω]_x
    omega_hat = [    0      -omega(3)  omega(2);
                  omega(3)      0     -omega(1);
                 -omega(2)  omega(1)      0   ];

    % se(3) 矩阵形式：
    % [ [ω]_×  v ]
    % [  0   0  ]
    %
    % se(3) matrix:
    % [ [ω]_x  v ]
    % [  0    0 ]
    se3mat = [omega_hat, v;
              0 0 0 0];
end
