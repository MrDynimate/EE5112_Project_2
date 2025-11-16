%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); 
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
Rinv = 2*Rinv/sum(sum(Rinv)); 


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    
    %% Sample noisy trajectories
[theta_samples, ~] = stompSamples(nPaths, Rinv, theta);   % Eq.(9)

%% TODO: Calculate Local trajectory cost for each sampled trajectory
Stheta = zeros(nPaths, nDiscretize);                      % K×N
for k = 1:nPaths
    [q_step, ~] = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
    Stheta(k, :) = q_step;                                % 1×N
end

Stheta = (Stheta - min(Stheta,[],1)) ./ (max(Stheta,[],1) - min(Stheta,[],1) + eps);

%% Given the local traj cost, update local trajectory probability
h = 25;  % 10~30 
Smin = min(Stheta, [], 1);                       % 1×N
Smax = max(Stheta, [], 1);                       % 1×N
den  = (Smax - Smin) + eps;                      % 1×N
W    = exp( -h * (Stheta - Smin) ./ den );       % K×N

[~, bestIdx] = min(Stheta, [], 1);               % 1×N
for i = 1:nDiscretize
    W(bestIdx(i), i) = W(bestIdx(i), i) * 1.1;
end
P = W ./ sum(W, 1);                             

%% Compute delta theta (aka gradient estimator)
dtheta = zeros(size(theta));                      % J×N
for i = 1:nDiscretize
    acc = zeros(numJoints,1);
    for k = 1:nPaths
        acc = acc + P(k,i) * (theta_samples{k}(:,i) - theta(:,i));
    end
    dtheta(:,i) = acc;
end

% （Eq.11）
dtheta_smoothed = zeros(size(dtheta));
dtheta_smoothed(:, 2:end-1) = (M * dtheta(:, 2:end-1)')';

% update
theta = theta + dtheta_smoothed;
theta(:,1)   = q0;
theta(:,end) = qT;

%% Compute the cost of the new trajectory
[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);



    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Record the itermediate training trajectories for later animation
    theta_animation{iter}=theta;

    % Set the stopping iteration criteria:
    if iter > 100 
        disp('Maximum iteration (50) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
    disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
    break
    end

end

disp('STOMP Finished.');






%% check collision
inCollision = false(nDiscretize, 1); % initialization the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");


    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
% Display whether there is collision:
isTrajectoryInCollision = any(inCollision)


%% Record the whole training/learning process in video file
enableVideoTraining = 1;

v = VideoWriter('KinvaGen3_Training.avi');
v.FrameRate = 15;
open(v);

htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);

if enableVideoTraining == 1
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);

    % safe index
    for idx = 1:5:nTraining
        UpdatedText = ['Iteration = ',num2str(idx)];
        set(htext,'String',UpdatedText)
        theta_tmp = theta_animation_tmp{idx};

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            frame = getframe(gcf);
            writeVideo(v,frame);
        end
        pause(1/15);
    end
end
close(v);


%% Record planned trajectory to video files
enableVideo = 0;
if enableVideo == 1
    v = VideoWriter('KinvaGen3_wEEConY3.avi');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
end
%% Show the planned trajectory
displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
end



%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')

