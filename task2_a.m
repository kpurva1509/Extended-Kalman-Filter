% (0, 0)

mean_r = 10.0;
mean_theta = 0.0;

num_points = 10000;

sd_r = 0.5;
sd_theta = 0.25;

r_values = mean_r + sd_r*randn(num_points, 1);
theta_values = mean_theta + sd_theta*randn(num_points, 1);

figure;
polarscatter(theta_values, r_values, '.');

x= r_values.*cos(theta_values);
y = r_values.*sin(theta_values);
figure; plot(x, y, '.');