function [mu, Sigma, predMu, predSigma, zhat] = ukfUpdate (mu, Sigma, u, deltaT, M, z, Q, markerId, w_mean, w_cov, lambda, t)

    % NOTE: The header is not set in stone.  You may change it if you like.
    global FIELDINFO;
    landmark_x = FIELDINFO.MARKER_X_POS(markerId);
    landmark_y = FIELDINFO.MARKER_Y_POS(markerId);
    
    num_aug_states = 3;
    
    n = 3 + num_aug_states;
    
    mu_aug = mu;
    sigma_aug = Sigma;
    sigma_aug(4:6, 4:6) = M;
    
    chi_t_m1 = generate_chi(mu_aug, n, lambda, sigma_aug);
    
    for index = 1:size(chi_t_m1,2)
        chi_t_star_bar(:, index) = prediction(chi_t_m1(:, index), u);
        chi_t_star_bar(3, index) = minimizedAngle(chi_t_star_bar(3, index));
    end
    
    mu_bar = zeros(n, 1);
    
    for index = 1:size(chi_t_star_bar,2)
        mu_bar(1:2) = mu_bar(1:2) + w_mean(index)*chi_t_star_bar(1:2, index);
        mu_bar(3) = mu_bar(3) + w_mean(index)*chi_t_star_bar(3, index);
        mu_bar(4:6) = mu_bar(4:6) + w_mean(index)*chi_t_star_bar(4:6, index);
    end
    
    sigma_aug_bar = zeros(n);
    
    for index = 1:size(chi_t_star_bar, 2)
        state_delta = chi_t_star_bar(:, index) - mu_bar;
        state_delta(3) = minimizedAngle(state_delta(3));
        sigma_aug_bar = sigma_aug_bar + w_cov(index)*(state_delta*state_delta');
    end
    
    theta = mu_bar(3);
    drot1 = u(1);
    d = u(2);
    drot2 = u(3);
    
    V_t = [ -d*sin(theta + drot1), cos(theta + drot1), 0;
        d*cos(theta + drot1), sin(theta + drot1), 0;
        1,  0, 1];
   
    R_t = V_t*M*V_t';
    
    sigma_aug_bar(1:3, 1:3) = sigma_aug_bar(1:3, 1:3) + R_t;
    
    chi_t_bar = generate_chi(mu_bar, n, lambda, sigma_aug_bar);
    
    z_t_bar = zeros(n, 1);
    
    for index = 1:2*n+1
        obs = observation(chi_t_bar(1:3, index), markerId);
        z_t_bar(index) = minimizedAngle(obs(1));
    end
    
    zhat = minimizedAngle(sum(dot(w_mean,z_t_bar)));
    
    S_t = 0;
    
    for index = 1:2*n+1
        delta_z = minimizedAngle(z_t_bar(index) - zhat);
        S_t = S_t + w_cov(index)*delta_z*delta_z';
    end
    
    S_t = S_t + Q;
    
    sigma_t_x_z_bar = zeros(n, 1);
    
    for index = 1:2*n+1
        delta_mu = chi_t_bar(:, index) - mu_bar;
        delta_mu(3) = minimizedAngle(delta_mu(3));
        delta_z = minimizedAngle(z_t_bar(index) - zhat);
        sigma_t_x_z_bar = sigma_t_x_z_bar + w_cov(index)*delta_mu*delta_z';
    end
    
    K_t = sigma_t_x_z_bar*inv(S_t);
    
    mu = mu_bar + K_t*(z - zhat);
    Sigma = sigma_aug_bar - K_t*S_t*K_t';
    
    predMu = mu_bar(1:3);
    predSigma = sigma_aug_bar(1:3, 1:3);
    zhat = z;
end