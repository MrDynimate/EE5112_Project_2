%% Visualize the obstacles and robot manipulator

% Initial visualizer 
positions = x0(1:numJoints)';

hgif = figure('Position', [375 446 641 480]);
ax1 = show(robot, positions(:,1), 'PreservePlot', false, 'Frames', 'off');
view(220,29);
hold on;
axis([-0.2 1.2 -0.6 0.7 0 1.2]);
title('Robot Manipulator and Obstacles Visualization');
grid on;

% ===== 可视化障碍物世界 =====
for i = 1:numel(world)
    [~, pObj] = show(world{i});
    pObj.LineStyle = 'none';
end

% ===== 可视化最终目标姿态与机械臂 =====
% 绘制目标末端位置点
plot3(poseFinal(1), poseFinal(2), poseFinal(3), 'r.', 'MarkerSize', 25);
text(poseFinal(1), poseFinal(2), poseFinal(3)+0.05, 'Target Pose', 'Color', 'r', 'FontSize', 10);

% 绘制目标姿态坐标系（替代 trplot）
origin = taskFinal(1:3, 4);
R = taskFinal(1:3, 1:3);
axisLength = 0.1;

% 绘制 X (红), Y (绿), Z (蓝) 方向箭头
quiver3(origin(1), origin(2), origin(3), R(1,1)*axisLength, R(2,1)*axisLength, R(3,1)*axisLength, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
quiver3(origin(1), origin(2), origin(3), R(1,2)*axisLength, R(2,2)*axisLength, R(3,2)*axisLength, 'g', 'LineWidth', 2, 'MaxHeadSize', 1);
quiver3(origin(1), origin(2), origin(3), R(1,3)*axisLength, R(2,3)*axisLength, R(3,3)*axisLength, 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
text(origin(1), origin(2), origin(3)+0.02, 'T_f', 'Color', 'k', 'FontSize', 10);

% 绘制最终姿态的机械臂
show(robot, finalRobotJConfig, 'PreservePlot', true, 'Frames', 'off', 'Parent', ax1);

% ===== 额外可视化优化 =====
% 可选：显示初始与目标末端连线
plot3([positions(1), poseFinal(1)], [positions(2), poseFinal(2)], [positions(3), poseFinal(3)], ...
      'k--', 'LineWidth', 1.2);

legend({'Final Pose','Obstacles','Target Frame','Final Robot'}, 'Location', 'northeastoutside');
