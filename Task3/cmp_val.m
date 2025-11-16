clc; clear; close all;

%% 1. 加载机器人模型 / Load robot model
robot_name = 'frankaEmikaPanda';
robot = loadrobot(robot_name, 'DataFormat', 'struct');

% 关节数（非固定关节） / Number of non-fixed joints
q_home = homeConfiguration(robot);
ndof   = numel(q_home);

bodyNames = robot.BodyNames;       % 所有刚体名 / all body names
baseName  = robot.BaseName;        % 参考刚体 / reference body

fprintf('Robot: %s, DoF (non-fixed): %d, Bodies: %d\n', ...
    robot_name, ndof, numel(bodyNames));

%% 2. 构造一批随机关节配置 / Build a set of random configurations

% 测试次数 / number of test configurations
Ncfg = 200;    

% 预分配 / preallocate
qList = cell(Ncfg, 1);

% 简单起见：对 revolute 关节在 [-pi/2, pi/2] 内随机，
% prismatic 这里不区分，仍用 [-0.2, 0.2] 之类范围（可按需要修改）
%
% For simplicity:
%   - revolute joints: random in [-pi/2, pi/2]
%   - prismatic: in [-0.2, 0.2] (adjust if needed)

for n = 1:Ncfg
    q = q_home;   % 从 home 拷贝一个 / copy from home
    for i = 1:ndof
        % 这里其实可以根据 robot.Bodies 的 Joint.Type 区分转动 / 平动
        % Here we could distinguish revolute / prismatic using robot.Bodies
        if true
            q(i).JointPosition = (rand()*pi - pi/2);  % [-pi/2, pi/2]
        end
    end
    qList{n} = q;
end

fprintf('Generated %d random configurations.\n', Ncfg);

%% 3. 数值正确性对比（在若干配置、若干刚体上） 
%    Numerical accuracy comparison

maxErr = 0;   % 记录所有测试中的最大 Frobenius 范数误差
              % record maximum Frobenius norm error

% 只抽几个 body 做展示，比如 hand、link7、finger 等
% pick a subset of bodies to print details
checkBodies = {'panda_link3', 'panda_link7', 'panda_hand'};

for n = 1:min(Ncfg, 20)   % 随机挑前 20 个配置做精细检查 / check first 20 configs
    q = qList{n};

    for k = 1:numel(bodyNames)
        bname = bodyNames{k};

        % 原版 PoE
        T1 = getTransformPoE_origin(robot, q, bname, baseName);
        % 优化版 PoE
        T2 = getTransformPoE(robot, q, bname, baseName);

        err = norm(T1 - T2, 'fro');
        maxErr = max(maxErr, err);

        % 若是我们关心的某些 body，就打印一次 / print some bodies for inspection
        if n == 1 && ismember(bname, checkBodies)
            fprintf('Config %d, body %s, ||T1-T2||_F = %.3e\n', n, bname, err);
        end
    end
end

fprintf('\nMax Frobenius error over checked configs/bodies: %.3e\n', maxErr);

%% 4. 运行时间对比 / Timing comparison

% 为了更稳定的计时，先预热一次 JIT / warm-up for JIT
qWarm = qList{1};
for k = 1:numel(bodyNames)
    bname = bodyNames{k};
    getTransformPoE_origin(robot, qWarm, bname, baseName);
    getTransformPoE(robot, qWarm, bname, baseName);
end

% 计时：原版 / timing: original
tic;
for n = 1:Ncfg
    q = qList{n};
    for k = 1:numel(bodyNames)
        bname = bodyNames{k};
        T1 = getTransformPoE_origin(robot, q, bname, baseName); %#ok<NASGU>
    end
end
t_origin = toc;

% 计时：优化版 / timing: optimized
tic;
for n = 1:Ncfg
    q = qList{n};
    for k = 1:numel(bodyNames)
        bname = bodyNames{k};
        T2 = getTransformPoE(robot, q, bname, baseName); %#ok<NASGU>
    end
end
t_opt = toc;

% 计时: 内置函数 / timing: getTransform()
tic;
for n = 1:Ncfg
    q = qList{n};
    for k = 1:numel(bodyNames)
        bname = bodyNames{k};
        T3 = getTransform(robot, q, bname, baseName); %#ok<NASGU>
    end
end
t_tool = toc;

fprintf('\n==== Timing over %d configs × %d bodies ====\n', Ncfg, numel(bodyNames));
fprintf('Original getTransformPoE_origin time : %.4f s\n', t_origin);
fprintf('Optimized getTransformPoE         time : %.4f s\n', t_opt);
fprintf('getTransform time : %.4f s\n', t_tool); 
fprintf('Speedup factor (origin / opt)        : %.2f x\n', t_origin / t_opt);

%% 5. 与内置 getTransform 再对比一个例子（可选）
%    Optional: compare PoE with Robotics Toolbox getTransform (sanity check)

q_test = qList{1};
bname  = 'panda_hand';

T_builtin = getTransform(robot, q_test, bname, baseName);
T_poe_opt = getTransformPoE(robot, q_test, bname, baseName);
T_poe = getTransformPoE_origin(robot, q_test, bname, baseName);

err_builtin = norm(T_builtin - T_poe_opt, 'fro');
err_origin = norm(T_builtin - T_poe, 'fro');
fprintf('\nCompare PoE vs toolbox getTransform for %s:\n', bname);
fprintf('||T_builtin - T_poe||_F = %.3e\n', err_origin);
fprintf('\nCompare optimized PoE vs toolbox getTransform for %s:\n', bname);
fprintf('||T_builtin - T_poe_opt||_F = %.3e\n', err_builtin);
