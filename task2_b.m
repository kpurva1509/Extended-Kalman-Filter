%(0,0)
function [x_y_mean, x_y_cov] = mapping(r_theta_mean, r_theta_cov, r_theta_point)

non_linear_at_mean = [r_theta_mean(1)*cos(r_theta_mean(2)); r_theta_mean(1)*sin(r_theta_mean(2))];

% Jacobian of the domain mapping from r-theta to x-y domain
derivative_at_point = [cos(r_theta_mean(2)) -1*r_theta_mean(1)*sin(r_theta_mean(2)); sin(r_theta_mean(2)) r_theta_mean(1)*cos(r_theta_mean(2))];
delta_point = r_theta_point - r_theta_mean;

x_y_mean = non_linear_at_mean + derivative_at_point*delta_point;

x_y_cov = derivative_at_point*r_theta_cov*derivative_at_point';
end

function task2_b
r_theta_mean = [10; 0];
r_theta_cov = [0.5^2, 0; 0, 0.25^2];
[linear_x_y_mean, linear_x_y_cov] = mapping(r_theta_mean, r_theta_cov, r_theta_mean);
display(linear_x_y_cov);

end

