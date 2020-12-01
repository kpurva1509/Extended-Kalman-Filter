function varargout = run(stepsOrData, pauseLen, makeVideo, filter)
% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN, MAKEVIDEO)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      MAKEVIDEO - boolean specifying whether to record a video or not
%
%   DATA = RUN(ARG,PAUSELEN)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.

%   (c) 2009-2015
%   Ryan M. Eustice
%   University of Michigan
%   eustice@umich.edu

if ~exist('pauseLen','var') || isempty(pauseLen)
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end
if ~exist('filter', 'var') || isempty(filter)
    filter = 1;
end

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';
TRACKED_PATH_COL = 'black';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

initialStateMean = [180 50 0]';

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.05 0.001 0.05 0.01].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(20);
beta = beta/2;

% Step size between filter updates, can be less than 1.
deltaT=0.1;

persistent data numSteps;
if isempty(stepsOrData) % use dataset from last time
    if isempty(data)
        numSteps = 100;
        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
    end
elseif isscalar(stepsOrData)
    % Generate a dataset of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
else
    % use a user supplied dataset from a previous run
    data = stepsOrData;
    numSteps = size(data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end


% TODO: provide proper initialization for your filters here
% You can set the initial mean and variance of the EKF to the true mean and
% some uncertainty.



% Call ekfUpdate, ukfUpdate and pfUpdate in every iteration of this loop.
% You might consider putting in a switch yard so you can select which
% algorithm does the update
results = [];

rng(0);

if filter==1
    state_vector_mean_ekf = initialStateMean;
    state_vector_sigma_ekf = 0.001*eye(3);
elseif filter==2
    state_vector_mean_ukf = [initialStateMean; zeros(3,1)];
    state_vector_sigma_ukf = 0.001*eye(6);
elseif filter==3
    numSamples_pkf = 100;
    samples_pkf = repmat(initialStateMean, 1, numSamples_pkf);
    for index = 1:numSamples_pkf
        samples_pkf(:, index) = sample(initialStateMean, 0.001*eye(3));
    end
    weight_pkf = ones(1, numSamples_pkf)*1/numSamples_pkf;
end

x_delta = zeros(1, numSteps);
y_delta = zeros(1, numSteps);
theta_delta = zeros(1, numSteps);

x_limits = zeros(1, numSteps);
y_limits = zeros(1, numSteps);
theta_limits = zeros(1, numSteps);

for t = 1:numSteps

    %=================================================
    % data available to your filter at this time step
    %=================================================
    motionCommand = data(t,3:5)'; % [drot1, dtrans, drot2]' noisefree control command
    observation = data(t,1:2)';   % [bearing, landmark_id]' noisy observation

    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================
    % actual position (i.e., ground truth)
    x = data(t,8);
    y = data(t,9);
    theta = data(t,10);

    % noisefree observation
    noisefreeBearing = data(t, 6);

    %=================================================
    % graphics
    %=================================================
    figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(2));

    % draw actual path and path that would result if there was no noise in
    % executing the motion command
    plot([initialStateMean(1) data(1,8)], [initialStateMean(2) data(1,9)], 'Color', ACTUAL_PATH_COL);
    plot([initialStateMean(1) data(1,11)], [initialStateMean(2) data(1,12)], 'Color', NOISEFREE_PATH_COL);

    % draw actual path (i.e., ground truth)
    plot(data(1:t,8), data(1:t,9), 'Color', ACTUAL_PATH_COL);
    plotrobot( x, y, theta, 'black', 1, 'cyan');

    % draw noise free motion command path
    plot(data(1:t,11), data(1:t,12), 'Color', NOISEFREE_PATH_COL);
    plot(data(t,11), data(t,12), '*', 'Color', NOISEFREE_PATH_COL);

    % indicate observed angle relative to actual position
    plot([x x+cos(theta+observation(1))*100], [y y+sin(theta+observation(1))*100], 'Color', OBSERVED_BEARING_COLOR);

    % indicate ideal noise-free angle relative to actual position
    plot([x x+cos(theta+noisefreeBearing)*100], [y y+sin(theta+noisefreeBearing)*100], 'Color', NOISEFREE_BEARING_COLOR);

    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    u = motionCommand;
    u(1) = minimizedAngle(u(1));
    u(3) = minimizedAngle(u(3));
    drot1 = u(1);
    dtran = u(2);
    drot2 = u(3);
    M = diag([alphas(1)*drot1^2+alphas(2)*dtran^2, alphas(3)*dtran^2+alphas(4)*(drot1^2+drot2^2), alphas(1)*drot2^2+alphas(2)*dtran^2]);
    z = minimizedAngle(observation(1));
    Q = beta;
    markerId = observation(2);
    
    if filter==1
        mu = state_vector_mean_ekf;
        Sigma = state_vector_sigma_ekf;
        [mu, Sigma, predMu, predSigma, zhat, G, R, H, K] = ekfUpdate(mu, Sigma, u, deltaT, M, z, Q, markerId, alphas);
        state_vector_mean_ekf = mu;
        state_vector_sigma_ekf = Sigma;
    elseif filter==2
        alpha = 1;
        kappa = 1;
        b = 1;
        m = 3;
        n = 3;
        lambda = alpha^2*(n+m+kappa) - n - m;
        w_mean = zeros(1, 2*(n+m)+1);
        w_cov = zeros(1, 2*(n+m)+1);
        i = 0;
        w_mean(i+1) = lambda/(n+m+lambda);
        w_cov(i+1) = lambda/(lambda+n+m)+(1 - alpha^2 + b);
        for i = 1:2*(n+m)
            w_mean(i+1) = 1/(2*(n+m+lambda));
            w_cov(i+1) = 1/(2*(n+m+lambda));
        end
        mu = state_vector_mean_ukf;
        Sigma = state_vector_sigma_ukf;
        [mu, Sigma, predMu, predSigma, zhat] = ukfUpdate(mu, Sigma, u, deltaT, M, z, Q, markerId, w_mean, w_cov, lambda, t);
        state_vector_mean_ukf = mu;
        state_vector_sigma_ukf = Sigma;        
        display(Sigma);
    elseif filter==3
        [samples_pkf, weight_pkf, mu, Sigma, predState, predMu, predSigma, zhat] = pfUpdate(samples_pkf, weight_pkf, numSamples_pkf, u, deltaT, M, z, Q, markerId, alphas);
        plotSamples(samples_pkf);
    end

    x_delta(t) = x - mu(1);
    y_delta(t) = y - mu(2);
    theta_delta(t) = theta - mu(3);
    
    x_limits(t) = 3*sqrt(Sigma(1,1));
    y_limits(t) = 3*sqrt(Sigma(2,2));
    theta_limits(t) = 3*sqrt(Sigma(3,3));
    
    draw_ellipse(mu(1:2), Sigma(1:2, 1:2), 4);
    draw_ellipse(predMu(1:2), predSigma(1:2, 1:2), 4);
    plotcov2d(mu(1), mu(2), Sigma(1:2, 1:2), 'b')
    
    draw_ellipse(mu(1:2), Sigma(1:2, 1:2), 9);
    draw_ellipse(predMu(1:2), predSigma(1:2, 1:2), 9);
    
    plot([mu(1) mu(1)+cos(mu(3)+ zhat)*100], [mu(2) mu(2)+sin(mu(3)+zhat)*100], 'Color', 'red');
    
    drawnow;
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end

if nargout >= 1
    varargout{1} = data;
end
if nargout >= 2
    varargout{2} = results;
end

figure; 
subplot(3,1,1);
plot(x_delta,'black', 'LineWidth', 2);
hold on; 
plot(x_limits,'red', 'LineWidth', 2);
hold on;
plot(-1*x_limits,'blue', 'LineWidth', 2);
legend('x_delta','x-max-limit', 'x-min-limit');
ylabel('ERROR')

subplot(3,1,2);
plot(y_delta,'black', 'LineWidth', 2);
hold on; 
plot(y_limits,'red', 'LineWidth', 2);
hold on;
plot(-1*y_limits,'blue', 'LineWidth', 2);
legend('y_delta','y-max-limit', 'y-min-limit');
ylabel('ERROR')

subplot(3,1,3);
plot(theta_delta,'black', 'LineWidth', 2);
hold on; 
plot(theta_limits,'red', 'LineWidth', 2);
hold on;
plot(-1*theta_limits,'blue', 'LineWidth', 2);
legend('theta_delta','theta-max-limit', 'theta-min-limit');
ylabel('ERROR')


if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
