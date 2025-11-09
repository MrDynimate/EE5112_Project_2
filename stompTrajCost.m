% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world)
% Compute the local trajectory cost at each discretization theta point, as 
% well as the overall trajecotry cost (the Qtheta)

% Costi = stompCompute_Cost(robot, theta, Env);
% Compute the cost of all discretized points on one trajectory
[~, nDiscretize] = size(theta);
% Obstacle cost
qo_cost = zeros(1, nDiscretize);
% Constraint costs
qc_cost = zeros(1, nDiscretize);

% Get the coordinates of joints in World frame 
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:, 1));
% Construct the spheres around the robot manipulator for collision
% avoidance
[sphere_centers,radi] = stompRobotSphere(X);
% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);

for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);
    % xb: 3D workspace position of sphere b at the current time
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
    qo_cost(i) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);
    
    %% Add end-effector orientation constraint (hand horizontal as defined)
    endEffectorName = 'panda_hand';  % 替换为你的末端执行器名称
    
    % --- Convert current column of theta to structure for getTransform ---
    configStruct = homeConfiguration(robot_struct);
    for j = 1:length(configStruct)
        configStruct(j).JointPosition = theta(j,i);
    end
    
    % --- Get end-effector transform ---
    T = getTransformPoE(robot_struct, configStruct, endEffectorName);
    R_ee = T(1:3,1:3);         % 末端旋转矩阵
    x_hand = R_ee(:,1);        % hand x轴
    z_hand = R_ee(:,3);        % hand z轴
    
    % --- Desired directions in base/world frame ---
    x_target = [0;0;-1];       % hand x 指向 base z 负方向
    z_target = [1;0;0];        % hand z 指向 base x 方向
    
    % --- Compute orientation cost ---
    w_ori = 5;                  % 姿态约束权重，可调
    qc_cost(i) = w_ori * (norm(x_hand - x_target,1) + norm(z_hand - z_target,1));  % L1范数惩罚
end

%% Local trajectory cost: you need to specify the relative weights between different costs
Stheta = 5000*qo_cost + 100*qc_cost;

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end
