clc;
clear;
close all;
delete('*.mat');
delete('*.asv');
delete('*.avi');

%% Load the model.
robot_name = 'frankaEmikaPanda';
robot = loadrobot(robot_name, 'DataFormat', 'column');
% Get the number of joints.
numJoints = numel(homeConfiguration(robot));
% Specify the robot frame where the end-effector is attached.
endEffector = "panda_hand";
% Joint Limit
qmin = [-2.897; -1.763; -2.897; -3.072; -2.897; -0.018; -2.897; 0; 0];
qmax = [ 2.897;  1.763;  2.897; -0.070;  2.897;  3.752;  2.897; 0.04; 0.04];

%% Specify initial and desired end-effector poses. Use inverse kinematics (maps from pose to joint angles) to solve for the initial robot configuration given a desired pose.
% 末端执行器位于机器人前方 0.5 m、高度 0.4 m 处，手爪朝下、x 轴反向（向后）、z 轴反向（向下）。

Xpos = 0.48;
Ypos = -0.2;
Zpos = 0.28;
T = trvec2tform([Xpos Ypos Zpos]);

yaw = pi/2;    % 绕Z轴旋转0°
pitch = pi/2;  % 绕Y轴旋转180°
roll = pi/2;   % 绕X轴旋转0°
R = eul2rotm([yaw pitch roll], 'ZYX');

taskInit = T;
taskInit(1:3,1:3) = R;   % 替换旋转

%% Compute current robot joint configuration using inverse kinematics
% 求机械臂逆解
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

% 输入ik的参数设置（指定末端关节，初始位置，权重，归位设置）
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
currentRobotJConfig = max(currentRobotJConfig, qmin);
currentRobotJConfig = min(currentRobotJConfig, qmax);

% The IK solver respects joint limits, but for those joints with infinite
% range, they must be wrapped to a finite range on the interval [-pi, pi].
% Since the the other joints are already bounded within this range, it is
% sufficient to simply call wrapToPi on the entire robot configuration
% rather than only on the joints with infinite range.
% 将无限旋转关节数据映射到[-pi,pi]的范围内，防止出错
% currentRobotJConfig = wrapToPi(currentRobotJConfig);

%% 末端执行器位于机器人前方 0.5 m、高度 0.4 m 处，手爪朝下、x 轴反向（向后）、z 轴反向（向下）。
Xposf = 0.65;
Yposf = 0.2;
Zposf = 0.58;
Tf = trvec2tform([Xposf Yposf Zposf]);

yawf = pi/2;    % 绕Z轴旋转0°
pitchf = pi/2;  % 绕Y轴旋转180°
rollf = pi/2;   % 绕X轴旋转0°
Rf = eul2rotm([yawf pitchf rollf], 'ZYX');

taskFinal = Tf;
taskFinal(1:3,1:3) = Rf;   % 替换旋转

anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
finalRobotJConfig = ik(endEffector, taskFinal, weights, currentRobotJConfig);
% finalRobotJConfig = wrapToPi(finalRobotJConfig);
finalRobotJConfig = max(finalRobotJConfig, qmin);
finalRobotJConfig = min(finalRobotJConfig, qmax);
%% Visualization
creatCollisionBox;

x0 = [currentRobotJConfig', zeros(1,numJoints)];
InitialVisualizer;
safetyDistance = 0.005; 

%% STOMP
helperSTOMP;