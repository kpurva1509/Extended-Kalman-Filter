% (0,0)

clear all;
close all;

corr_coefficient = [0.1, 0.5, 0.9];
sd_r = 0.5;
sd_theta = 0.25;

for corr_coeff = corr_coefficient
    mean_r = 10.0;
    mean_theta = 0.0;
    num_points = 10000;
    sigma_r_theta = [ sd_r^2, sd_r*sd_theta*corr_coeff; sd_r*sd_theta*corr_coeff, sd_theta^2];
    mean_r_theta = [mean_r; mean_theta];
    for index=1:num_points
        actual_r_theta_values(:, index) = mvnrnd(mean_r_theta, sigma_r_theta);    
    end
    actual_x_values = actual_r_theta_values(1,:).*cos(actual_r_theta_values(2, :));
    actual_y_values = actual_r_theta_values(1,:).*sin(actual_r_theta_values(2, :));
    
    figure;
    plot(actual_x_values, actual_y_values, '.');
    xlim([0, 15]);
    ylim([-10, 10]);
    hold on;
    actual_mean = [mean(actual_x_values); mean(actual_y_values)];
    actual_var_x = var(actual_x_values);
    actual_var_y = var(actual_y_values);
    actual_sigma = cov(actual_x_values, actual_y_values);
    
    x_y_values = [];
    for index=1:num_points
        [linear_x_y_mean, linear_x_y_cov] = mapping([mean_r; mean_theta], sigma_r_theta, actual_r_theta_values(:, index));
        x_y_values(:, index) = linear_x_y_mean;
    end

    linearized_var_x = var(x_y_values(1, :));
    linearized_var_y = var(x_y_values(2, :));
    linearized_sigma = [linearized_var_x, 0; 0, linearized_var_y];

    linearized_mean_x = mean(x_y_values(1, :));
    linearized_mean_y = mean(x_y_values(2, :));
    
    figure;
    plot(x_y_values(1,:), x_y_values(2,:), '.');
    hold on;
    xlim([0, 15]);
    ylim([-10, 10]);

    draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 1, 'color','red','linewidth',1.5);
    draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 4, 'color','red','linewidth',1.5);
    draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 9, 'color','red','linewidth',1.5);
    draw_ellipse([actual_mean(1); actual_mean(2)], actual_sigma, 1, 'color','blue','linewidth',1.5);
    draw_ellipse([actual_mean(1); actual_mean(2)], actual_sigma, 4, 'color','blue','linewidth',1.5);
    draw_ellipse([actual_mean(1); actual_mean(2)], actual_sigma, 9, 'color','blue','linewidth',1.5);


end