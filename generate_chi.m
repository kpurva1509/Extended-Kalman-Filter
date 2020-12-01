function [chi] = generate_chi(mu_aug, n, lambda, sigma_aug)
    %GENERATE_CHI Summary of this function goes here
    %   Detailed explanation goes here
    chi = zeros(n, 2*n+1);
    
    chi(:, 1) = mu_aug;
    chi(3, 1) = minimizedAngle(chi(3, 1));
    gamma = sqrtm((lambda+n)*sigma_aug);
    
    for index = 1:2*n
        if index <= n
            chi(:, index+1) = mu_aug + gamma(:, index);
        else
            chi(:, index+1) = mu_aug - gamma(:, index-n);
        end
        chi(3, index) = minimizedAngle(chi(3, index));
    end
end

