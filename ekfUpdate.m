function [mu, Sigma, predMu, predSigma, zhat, G, R, H, K ] = ekfUpdate( ...
    mu, Sigma, u, deltaT, M, z, Q, markerId, alphas)

% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;

% --------------------------------------------
% Prediction step
% --------------------------------------------
mu_bar = prediction(mu, u);

x = mu_bar(1);
y = mu_bar(2);
theta = mu_bar(3);

drot1 = u(1);
d = u(2);
drot2 = u(3);

G_t = [1, 0, -d*sin(theta + drot1);
       0, 1,  d*cos(theta + drot1);
       0, 0,  1];

V_t = [ -d*sin(theta + drot1), cos(theta + drot1), 0;
        d*cos(theta + drot1), sin(theta + drot1), 0;
        1,  0, 1];
   
R_t = V_t*M*V_t';

sigma_bar = G_t*Sigma*G_t' + R_t;
display(sigma_bar);

temp = observation(mu_bar, markerId);
h_t = temp(1);

q = (landmark_x - x)^2 + (landmark_y - y)^2;
H_t = [(landmark_y - y)/q, -(landmark_x - x)/q, -1];

K_t = sigma_bar*H_t'/((H_t*sigma_bar*H_t' + Q));

% EKF prediction of mean and covariance
mu = mu_bar + K_t*(z - h_t);
Sigma = (eye(3) - K_t*H_t)*sigma_bar;
display(Sigma);

mu(3) = minimizedAngle(mu(3));

predMu = mu_bar;
predSigma = sigma_bar;
zhat = h_t;
G = G_t;
R = R_t;
H = H_t;
K = K_t;
return

% Sigma =
% 
%   272.7412   11.8692    0.7322
%    11.8692  214.6394    0.3953
%     0.7322    0.3953    0.0097
% 
% 
% Sigma =
% 
%   272.4152   19.4432    0.7340
%    19.4432  222.3563    0.4794
%     0.7340    0.4794    0.0098

