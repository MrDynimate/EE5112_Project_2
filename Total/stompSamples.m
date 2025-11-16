% Input: 
%   nSamplePaths: the number of sample paths 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma));

%% TODO: complete code of independent sampling for each joint
nInner    = nDiscretize - 2;                  % 只对中间列采样
mu_inner  = zeros(1, nInner);

% 统一得到中间段协方差 (nInner×nInner)，并做数值稳健
if size(sigma,1) == nInner
    Sigma_inner = sigma;
else
    Sigma_inner = sigma(2:end-1, 2:end-1);
end
Sigma_inner = (Sigma_inner + Sigma_inner')/2 + 1e-8*eye(nInner);  % 对称+抖动

explore_scale = 3;   % 2.0~3.0 之间调，增强探索

for m = 1:nJoints
    % 采样中间列 (K × nInner)，并放大探索
    noise_inner = mvnrnd(mu_inner, Sigma_inner, nSamplePaths) * explore_scale;

    % 拼回完整长度 (K × N)，首尾固定为 0
    noise_full  = zeros(nSamplePaths, nDiscretize);
    noise_full(:, 2:end-1) = noise_inner;

    em{m} = noise_full;   % 保存该关节的噪声
end

%% Regroup it by samples
emk = cat(2, em{:});                         % K × (N*J)  
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:), [nJoints, nDiscretize]);  % J×N
    theta_paths{k} = theta + ek{k};
end

