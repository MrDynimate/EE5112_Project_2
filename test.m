robot_name = 'frankaEmikaPanda';
robot = loadrobot(robot_name);
for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;
    if ~strcmp(joint.Type, 'fixed')   % 只关心非固定关节
        fprintf('Joint %s: min=%.3f, max=%.3f rad\n', ...
                joint.Name, joint.PositionLimits(1), joint.PositionLimits(2));
    end
end
