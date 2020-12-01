function [samples, weight, mu, Sigma, predState, predMu, predSigma, zHat] = pfUpdate(samples, weight, numSamples, u, deltaT, M, z, Q, markerId, alphas)

% NOTE: The header is not set in stone.  You may change it if you like
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;

zhat_observations = zeros(size(samples, 2), 1);
newSamples = zeros(size(samples));
for index = 1:numSamples
    newSamples(:, index) = sampleOdometry(u, samples(:, index), alphas);
    zhat = observation(newSamples(:, index), markerId);
    zhat = minimizedAngle(zhat(1));
    zhat_observations(index) = zhat;    
    weight(index) = normpdf(zhat, z, sqrtm(Q));
end

weight = weight/sum(weight);
zHat = dot(weight, zhat_observations);

% Resample here based on the weights
[samples, weight] = resample(newSamples, weight);

[predMu, predSigma] = meanAndVariance(samples, numSamples);

mu = predMu;
Sigma = predSigma;
predState = mu;


