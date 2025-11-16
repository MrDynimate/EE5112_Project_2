%% Visualize the obstacles and robot manipulator

% Initial visualizer 
positions = x0(1:numJoints)';
hgif = figure('Position', [375 446 641 480]);
ax1 = axes(hgif);
view(220,29);
% add light
light('Parent', ax1, 'Position', [1 1 2], 'Style', 'infinite'); % light 1
% light('Parent', ax1, 'Position', [-1 -1 0.5], 'Style', 'infinite'); % light 2

% light-material
material(ax1, 'metal');   % 'shiny', 'metal', 'default'

axis vis3d;           % fix axis
camproj('perspective'); 
show(robot, positions(:,1), 'PreservePlot', false, 'Frames', 'off');

hold on;
axis([-0.2 1.2 -0.6 0.7 0 1.2]);
title('Robot Manipulator and Obstacles Visualization');
grid on;

% plot3(poseInit(1), poseInit(2), poseInit(3), 'r.', 'MarkerSize', 25);
% text(poseInit(1), poseInit(2), poseInit(3)+0.05, 'Init Pose', 'Color', 'r', 'FontSize', 10);

% ===== Visualize the obstacles =====
for i = 1:numel(world)
    [~, pObj] = show(world{i});
    pObj.LineStyle = 'none';
end

% ===== Visualize the  manipulator =====
% target point
plot3(poseFinal(1), poseFinal(2), poseFinal(3), 'r.', 'MarkerSize', 25);
text(poseFinal(1), poseFinal(2), poseFinal(3)+0.05, 'Target Pose', 'Color', 'r', 'FontSize', 10);

origin = taskFinal(1:3, 4);
R = taskFinal(1:3, 1:3);
axisLength = 0.1;

% plot X Y Z
quiver3(origin(1), origin(2), origin(3), R(1,1)*axisLength, R(2,1)*axisLength, R(3,1)*axisLength, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
quiver3(origin(1), origin(2), origin(3), R(1,2)*axisLength, R(2,2)*axisLength, R(3,2)*axisLength, 'g', 'LineWidth', 2, 'MaxHeadSize', 1);
quiver3(origin(1), origin(2), origin(3), R(1,3)*axisLength, R(2,3)*axisLength, R(3,3)*axisLength, 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
text(origin(1), origin(2), origin(3)+0.02, 'T_f', 'Color', 'k', 'FontSize', 10);

% show finalRobot
show(robot, finalRobotJConfig, 'PreservePlot', true, 'Frames', 'off', 'Parent', ax1);

plot3([positions(1), poseFinal(1)], [positions(2), poseFinal(2)], [positions(3), poseFinal(3)], ...
      'k--', 'LineWidth', 1.2);

legend({'Final Pose','Obstacles','Target Frame','Final Robot'}, 'Location', 'northeastoutside');
