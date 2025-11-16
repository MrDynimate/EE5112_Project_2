function qc = stompOrientationCost(robot_struct, theta_i, endEffectorName)
    % Convert current joint vector to structure format for getTransform
    configStruct = homeConfiguration(robot_struct);
    for j = 1:length(configStruct)
        configStruct(j).JointPosition = theta_i(j);
    end

    % Compute end-effector transformation
    T = getTransformPoE(robot_struct, configStruct, endEffectorName);
    R_ee = T(1:3,1:3);         % End-effector rotation matrix
    x_hand = R_ee(:,1);        % Hand x-axis
    z_hand = R_ee(:,3);        % Hand z-axis

    % Desired hand directions in the world frame
    x_target = [0;0;-1];       % Hand x-axis should point along -Z_base
    z_target = [1;0;0];        % Hand z-axis should point along +X_base

    % Compute L1 norm orientation deviation cost
    w_ori = 5;  % Orientation weight
    qc = w_ori * (norm(x_hand - x_target, 1) + norm(z_hand - z_target, 1));
end
