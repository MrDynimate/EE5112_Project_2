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
nInner    = nDiscretize - 2;                  % sample only interior waypoints
mu_inner  = zeros(1, nInner);                 % zero-mean perturbations

% Get the interior covariance (nInner x nInner) and make it numerically stable
if size(sigma,1) == nInner
    Sigma_inner = sigma;
else
    Sigma_inner = sigma(2:end-1, 2:end-1);
end
Sigma_inner = (Sigma_inner + Sigma_inner')/2 + 1e-8*eye(nInner);   % symmetrize + jitter

explore_scale = 3;   % exploration strength (typical 2.0–3.0)

for m = 1:nJoints
    % Draw K samples for the interior waypoints, then pad zeros at endpoints
    % noise_inner: K x nInner
    noise_inner = mvnrnd(mu_inner, Sigma_inner, nSamplePaths) * explore_scale;
    
    % Reconstruct full-length noise with fixed start/end (K x N)
    noise_full  = zeros(nSamplePaths, nDiscretize);
    noise_full(:, 2:end-1) = noise_inner;

    em{m} = noise_full;      % Store per-joint noise matrix (K x N)
end

%% Regroup it by samples
emk = cat(2, em{:});                         % K × (N*J)  
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:), [nJoints, nDiscretize]);  % J×N
    theta_paths{k} = theta + ek{k};
end

