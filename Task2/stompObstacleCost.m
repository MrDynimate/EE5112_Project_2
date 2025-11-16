function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
try
    % Clamp indices inside world boundaries
    idx(:,1) = min(max(idx(:,1), 1), world_size(1));
    idx(:,2) = min(max(idx(:,2), 1), world_size(2));
    idx(:,3) = min(max(idx(:,3), 1), world_size(3));

    % Convert subscript indices to linear index
    linear_idx = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));

    % Signed Euclidean distance at each sphere center
    dist_vals = voxel_world_sEDT(linear_idx);

    % Eq.(13): q_o = Σ_b max(ε + r_b - d(x_b), 0) * ||v_b||
    penetration = max(safety_margin + radius - dist_vals, 0);  % penalty term
    vel_mag = sqrt(sum(vel.^2, 2));                             % ||v_b||
    cost_array = penetration .* vel_mag;                        % elementwise multiply
    cost = sum(cost_array);
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end