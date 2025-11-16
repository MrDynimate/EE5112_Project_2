function T = getTransformPoE(robot, q, bodyName, baseName)
% GETTRANSFORMPOE  用指数积（PoE）方法计算刚体间位姿
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
        error('Only DataFormat="struct" is supported.');
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
% Compute ^{BaseName}T_{eeName} using PoE at configuration q.

    rootName = robot.BaseName;

    % 如果目标就是根节点，返回单位阵
    if strcmp(eeName, rootName)
        T = eye(4);
        return;
    end

    % ========= 1. 使用 persistent 缓存几何参数 / cache geometry =========
    persistent poeCache;

    % 原始 key，可以随便拼字符串
    % original key as string
    rawKey = sprintf('%s|%s', char(eeName), rootName);
    
    % 转成合法的结构体字段名
    % convert to a valid struct field name
    key = matlab.lang.makeValidName(rawKey);
    
    needBuild = true;
    if ~isempty(poeCache) && isfield(poeCache, key)
        entry = poeCache.(key);
        if entry.NumBodies == robot.NumBodies
            needBuild = false;
            Slist      = entry.Slist;
            jointNames = entry.jointNames;
            M          = entry.M;
        end
    end

    if needBuild
        % ---------- 首次构造：build Slist, jointNames, M --------------
        [bodyChain, ~] = getBodyJointChain(robot, eeName);

        % 先统计一下有多少非 fixed 关节，预分配 Slist / jointNames
        % First count non-fixed joints to preallocate
        ndof = 0;
        for i = 1:numel(bodyChain)
            b = getBody(robot, bodyChain{i});
            if ~strcmp(b.Joint.Type, 'fixed')
                ndof = ndof + 1;
            end
        end

        Slist      = zeros(6, ndof);
        jointNames = cell(1, ndof);
        T_home     = eye(4);

        idx = 0;
        for i = 1:numel(bodyChain)
            bname = bodyChain{i};
            body  = getBody(robot, bname);

            A_i   = body.Joint.JointToParentTransform;
            T_home = T_home * A_i;

            if strcmp(body.Joint.Type, 'fixed')
                continue;   % fixed 关节只影响 T_home，不产生 twist
            end

            % 当前关节原点和姿态（root frame） / joint origin & orientation in root frame
            R = T_home(1:3, 1:3);
            p = T_home(1:3, 4);

            axis_local = body.Joint.JointAxis(:);

            if strcmp(body.Joint.Type, 'revolute')
                omega = R * axis_local;
                v     = -cross(omega, p);
            else % prismatic
                omega = [0; 0; 0];
                v     = R * axis_local;
            end

            idx = idx + 1;
            Slist(:, idx)      = [omega; v];
            jointNames{idx} = body.Joint.Name;
        end

        M = T_home;  % 零位末端位姿 / home configuration pose

        % 写入缓存 / save to cache
        entry.Slist      = Slist;
        entry.jointNames = jointNames;
        entry.M          = M;
        entry.NumBodies  = robot.NumBodies;
        poeCache.(key)   = entry;
    end

    % ========= 2. 从 q 中提取 thetalist / extract thetalist ============
    ndof      = numel(jointNames);
    thetalist = zeros(ndof, 1);
    qNames    = {q.JointName};

    for i = 1:ndof
        jname = jointNames{i};
        idx = find(strcmp(qNames, jname), 1);
        if isempty(idx)
            error('Joint "%s" not found in q.', jname, jname);
        end
        thetalist(i) = q(idx).JointPosition;
    end

    % ========= 3. 使用 PoE 公式计算 / apply PoE ========================
    T = eye(4);
    for i = 1:ndof
        % use analytic se(3) exponential
        T = T * se3Exp(Slist(:, i), thetalist(i));
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

function T = se3Exp(S, theta)
%SE3EXP  计算 se(3) 指数 e^[S]θ，对刚体运动有解析解
%SE3EXP  Compute exp([S]^ ∧ θ) for se(3) twist S

    omega = S(1:3);
    v     = S(4:6);

    w_norm = norm(omega);

    if w_norm < 1e-8
        % 视为纯平移 / treat as pure translation
        R = eye(3);
        p = v * theta;
    else
        % 假设 omega 已经是单位向量，这里也可以做正规化
        % Assume omega is unit; normalize if needed
        w = omega / w_norm;
        wx = [   0   -w(3)  w(2);
               w(3)    0   -w(1);
              -w(2)  w(1)   0  ];

        R = eye(3) + sin(theta)*wx + (1-cos(theta))*(wx*wx);

        G = eye(3)*theta + (1-cos(theta))*wx + (theta - sin(theta))*(wx*wx);
        p = G * v;
    end

    T = [R, p;
         0 0 0 1];
end

