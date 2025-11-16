%% 1️⃣ 定义工作空间
Env_size = [-1, -1, -1; 2, 2, 2];  % [xmin, ymin, zmin] + xyz-length
voxel_size = [0.02, 0.02, 0.02];   % 每个体素的分辨率 (m)

% 初始化占据矩阵 (0=空闲, 1=占据)
binary_world = zeros(Env_size(2,1)/voxel_size(1), ...
                     Env_size(2,2)/voxel_size(2), ...
                     Env_size(2,3)/voxel_size(3));

[Xw, Yw, Zw] = meshgrid( ...
    Env_size(1,1)+0.5*voxel_size(1):voxel_size(1):Env_size(1,1)+Env_size(2,1)-0.5*voxel_size(1), ...
    Env_size(1,2)+0.5*voxel_size(2):voxel_size(2):Env_size(1,2)+Env_size(2,2)-0.5*voxel_size(2), ...
    Env_size(1,3)+0.5*voxel_size(3):voxel_size(3):Env_size(1,3)+Env_size(2,3)-0.5*voxel_size(3));


%% Object Boundary Function Definition
voxelizeBox = @(center, size) [center - size/2; size];
voxelizeCyl = @(center, r, h) [center(1)-r, center(2)-r, center(3)-h/2; 2*r, 2*r, h];
voxelizeSphere = @(center, r) [center-[r r r]; 2*r, 2*r, 2*r];

%% Scene Object Parameter Definition

% desk
desk_size = [0.8, 0.8, 0.05];
desk_center = [0.8, 0, 0.5];
% oven
oven_size = [0.48, 0.5, 0.24];
oven_center = [0.8, 0, 0.12];
% support block
block_size = [0.2, 0.2, 0.3];
block_center = [0.8, 0.15, 0.35];
% dish
dish_r = 0.07;
dish_center = [0.5, 0.05, 0.55];
% bottle down
r1 = 0.04; h1 = 0.15;
cyl_bottom_center = [0.5, -0.1, 0.5 + h1/2];
% bottle up
r2 = 0.02; h2 = 0.10;
cyl_top_center = [0.5, -0.1, 0.5 + h1 + h2/2];

%% Establishing Boundaries (xmin, ymin, zmin; dx, dy, dz)
objects = {
    voxelizeBox(desk_center, desk_size)
    voxelizeBox(oven_center, oven_size)
    voxelizeBox(block_center, block_size)
    voxelizeSphere(dish_center, dish_r)
    voxelizeCyl(cyl_bottom_center, r1, h1)
    voxelizeCyl(cyl_top_center, r2, h2)
};

%% 添加进world数组
world = {
    collisionBox(desk_size(1), desk_size(2), desk_size(3))
    collisionBox(oven_size(1), oven_size(2), oven_size(3))
    collisionBox(block_size(1), block_size(2), block_size(3))
    collisionSphere(dish_r)
    collisionCylinder(r1, h1)
    collisionCylinder(r2, h2)
};

% 设置每个障碍物的位姿
centers = {desk_center, oven_center, block_center, dish_center, cyl_bottom_center, cyl_top_center};
for i = 1:length(world)
    world{i}.Pose = trvec2tform(centers{i});
end
%% Voxelize each object into an occupancy matrix
for i = 1:length(objects)
    cube_metric = objects{i};
    % Convert to voxel index
    cube_voxel = [ceil((cube_metric(1,:) - Env_size(1,:)) ./ voxel_size); ...
                  ceil((cube_metric(1,:) + cube_metric(2,:) - Env_size(1,:)) ./ voxel_size)];

    % Generate Index Grid
    [xc, yc, zc] = meshgrid( ...
        cube_voxel(1,1):cube_voxel(2,1), ...
        cube_voxel(1,2):cube_voxel(2,2), ...
        cube_voxel(1,3):cube_voxel(2,3) ...
    );

    % Marking occupies voxels
    valid = (xc>0 & yc>0 & zc>0 & ...
             xc<=size(binary_world,1) & ...
             yc<=size(binary_world,2) & ...
             zc<=size(binary_world,3));
    ind = sub2ind(size(binary_world), xc(valid), yc(valid), zc(valid));
    binary_world(ind) = 1;
end
%% 6️⃣ 计算Signed Euclidean Distance Transform
voxel_world_sEDT = prod(voxel_size)^(1/3) * sEDT_3d(binary_world);

%% 7️⃣ 打包voxel world结构体
voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size;
voxel_world.sEDT = voxel_world_sEDT;
