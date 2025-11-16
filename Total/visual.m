% index
[x_idx, y_idx, z_idx] = ind2sub(size(binary_world), find(binary_world));

% tran
X = (x_idx-0.5)*voxel_size(1) + Env_size(1,1);
Y = (y_idx-0.5)*voxel_size(2) + Env_size(1,2);
Z = (z_idx-0.5)*voxel_size(3) + Env_size(1,3);

figure;
hold on;
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3)
title('Voxelized Environment (star representation)');

% plot '*'
for i = 1:length(X)
    text(X(i), Y(i), Z(i), '*', 'FontSize', 8, 'Color', [0 0 0.8]);
end
