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

%% 

% Init position
Xpos = 0.48;
Ypos = -0.2;
Zpos = 0.28;
T = trvec2tform([Xpos Ypos Zpos]);
% Orientation
yaw = pi/2;    
pitch = pi/2;  
roll = pi/2;   
R = eul2rotm([yaw pitch roll], 'ZYX');
% Init task-space pose
taskInit = T;
taskInit(1:3,1:3) = R;   % Insert rotation matrix into transformation matrix

anglesInit = rotm2eul(taskInit(1:3,1:3),'XYZ');
poseInit = [taskInit(1:3,4);anglesInit']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]
%% Compute current robot joint configuration using inverse kinematics
% 
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

% ik Config
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
% keep in Joint Limit
currentRobotJConfig = max(currentRobotJConfig, qmin);
currentRobotJConfig = min(currentRobotJConfig, qmax);

%% Target
% Target position
Xposf = 0.65;
Yposf = 0.2;
Zposf = 0.58;
Tf = trvec2tform([Xposf Yposf Zposf]);

% Orientation
yawf = pi/2;    
pitchf = pi/2;  
rollf = pi/2;   
Rf = eul2rotm([yawf pitchf rollf], 'ZYX');

% Target task-space pose
taskFinal = Tf;
taskFinal(1:3,1:3) = Rf;   % Insert rotation matrix into transformation matrix

anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
finalRobotJConfig = ik(endEffector, taskFinal, weights, currentRobotJConfig);
%Apply joint limits
finalRobotJConfig = max(finalRobotJConfig, qmin);
finalRobotJConfig = min(finalRobotJConfig, qmax);
%% Visualization
creatCollisionBox;

x0 = [currentRobotJConfig', zeros(1,numJoints)];
InitialVisualizer;
safetyDistance = 0.005; 

%% STOMP
helperSTOMP;