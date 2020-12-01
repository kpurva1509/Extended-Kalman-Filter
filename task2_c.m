clear all;
close all;
mean_r = 10.0;
mean_theta = 0.0;

num_points = 10000;

sd_r = 0.5;
sd_theta = 0.25;

for index=1:num_points
    r_values(index) = sample(mean_r, sd_r^2);
    theta_values(index) = sample(mean_theta, sd_theta^2);    
end

actual_x_values = r_values.*cos(theta_values);
actual_y_values = r_values.*sin(theta_values);

actual_var_x = var(actual_x_values);
actual_var_y = var(actual_y_values);
actual_sigma = [actual_var_x, 0; 0, actual_var_y];

for index=1:num_points
    [linear_x_y_mean, linear_x_y_cov] = mapping([mean_r; mean_theta], [sd_r^2 0; 0, sd_theta^2], [r_values(index); theta_values(index)]);
    x_y_values(:, index) = linear_x_y_mean;
end

linearized_var_x = var(x_y_values(1, :));
linearized_var_y = var(x_y_values(2, :));
linearized_sigma = [linearized_var_x, 0; 0, linearized_var_y];

linearized_mean_x = mean(x_y_values(1, :));
linearized_mean_y = mean(x_y_values(2, :));

figure;
plot(x_y_values(1,:), x_y_values(2,:), '.');
xlim([0 15]);
ylim([-10 10]);
hold on;

draw_ellipse([linearized_mean_x; linearized_mean_y], [actual_var_x, 0; 0, actual_var_y], 1, 'color','blue','linewidth',1.5);
draw_ellipse([linearized_mean_x; linearized_mean_y], [actual_var_x, 0; 0, actual_var_y], 4, 'color','blue','linewidth',1.5);
draw_ellipse([linearized_mean_x; linearized_mean_y], [actual_var_x, 0; 0, actual_var_y], 9, 'color','blue','linewidth',1.5);


draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 1, 'color','red','linewidth',1.5);
draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 4, 'color','red','linewidth',1.5);
draw_ellipse([linearized_mean_x; linearized_mean_y], [linearized_var_x, 0; 0, linearized_var_y], 9, 'color','red','linewidth',1.5);
