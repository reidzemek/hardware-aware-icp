%% Point-to-point Iterative Closest Point Algorithm - 2D

% 2-dimensional hardware friendly implementation of the point-to-point 
% Iterative Closest Point (ICP) point cloud registration algorithm

out_dir = 'P2P_ICP_2D';

%% Generate an irregular smooth squiggly function

% Parameters
range = 800;            % size of 2D square containing line
extent = 8;             % factor to scale range and define the horizontal extent of the line
n_points = 10;          % number of horizontal points per range increment
downsample_factor = 4;  % downsample factor for a single range increment

% Check that downsample_factor is a factor of range/extent
if mod(range / extent, downsample_factor) ~= 0
    error('downsample_factor must be a factor of range/extent');
end

% Adjust downsample factor based on extent
downsample_factor = downsample_factor * extent;

% Initialize time signal vector for horizontal axis (x)
t = linspace(0, extent*range, (range+1)*n_points);
y = zeros(size(t));

% Number of harmonic components (more terms = more irregularity)
num_terms = 100;

for k = 1:num_terms
    A = ((rand()*(2*range/3))+(range/3))*(1/k);
    f = (rand()*1/range)+1/(3*range);
    phi = rand()*2*pi;
    if rand() > 0.5
        y = y + A * sin(2*pi*f*t + phi);
    else
        y = y + A * cos(2*pi*f*t + phi);
    end
end

% Densely sampled 2D point cloud
xy = [t' y'];

%% Downsampled 2D Point Cloud

% Downsample and add an x offset
offset = rand()*range-(range/2);
xy(:, 1) = xy(:, 1) + offset;
pc = xy(1:range/downsample_factor:end, :);

% Define corners for a bounding box around pc with a 20% border
border = min([0.2*(max(pc(:, 1))-min(pc(:, 1))), 0.2*(max(pc(:, 2))-min(pc(:, 2)))]);
cnrs = [min(pc(:, 1))-border, min(pc(:, 2))-border; ...
        max(pc(:, 1))+border, min(pc(:, 2))-border; ...
        max(pc(:, 1))+border, max(pc(:, 2))+border; ...
        min(pc(:, 1))-border, max(pc(:, 2))+border];
cnrs(end+1,:) = cnrs(1,:);

%% Transform the point cloud and add some noise

% Generate random rotation angle (in radians)
theta = rand() * 2 * pi;

% 2D rotation matrix
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];

% Random translation vector (1x2)
t = rand(1, 2)*2*range-range;

% Add Gaussian noise to the point cloud
noise_level = range/35; % Adjust the noise level as needed
noise = noise_level * randn(size(pc)); % Generate Gaussian noise
pc_noisy = pc + noise; % Add noise to the point cloud

% Apply transformation: rotate then translate
pc_transformed = (R * pc_noisy')' + t;
xy_transformed = (R * xy')' + t;
cnrs_transformed = (R * cnrs')' + t;

%% Visualize point clouds

% Output directory
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% Plot original function
figure; hold on;
plot(xy(:, 1), xy(:, 2), 'LineWidth', 0.1, 'Color', repmat(0.8, [1 3]));
scatter(pc(:, 1), pc(:, 2), 10, 'filled', 'MarkerFaceColor', repmat(0.5, [1 3]));
hold off;
axis equal;
grid on; grid minor;
title('Irregular Smooth Squiggly Function');
fig1 = gcf;

% Visualize point clouds before ICP
figure; hold on;

% Original point cloud
scatter(pc(:, 1), pc(:, 2), 10, 'filled', 'MarkerFaceColor', repmat(0.5, [1 3]));
plot(xy(:, 1), xy(:, 2), 'LineWidth', 0.1, 'Color', repmat(0.8, [1 3]));
plot(cnrs(:, 1), cnrs(:, 2), 'LineWidth', 0.1, 'Color', 'r');

% Transformed point cloud
scatter(pc_transformed(:, 1), pc_transformed(:, 2), 10, 'filled', 'MarkerFaceColor', [0.5 0.5 0.5]);
plot(xy_transformed(:, 1), xy_transformed(:, 2), 'LineWidth', 0.1, 'Color', repmat(0.8, [1 3]));
plot(cnrs_transformed(:, 1), cnrs_transformed(:, 2), 'LineWidth', 0.1, 'Color', 'r'); hold off;

% Get current axis limits
x_limits = xlim;
y_limits = ylim;

% Increase limits by 20%
x_range_increase = 0.05 * (x_limits(2) - x_limits(1));
y_range_increase = 0.05 * (y_limits(2) - y_limits(1));

% Set new axis limits
xlim([x_limits(1) - x_range_increase, x_limits(2) + x_range_increase]);
ylim([y_limits(1) - y_range_increase, y_limits(2) + y_range_increase]);

axis equal;
grid on; grid minor;
title(sprintf("Rotation: %.2f°, Translation: %.2f, %.2f", rad2deg(theta), t(1), t(2)));
fig2 = gcf;

%% Prompt the user to save figures

saveFiguresInput = input('Set save figures flag? (Y/N): ', 's');

if strcmpi(saveFiguresInput, 'Y')
    saveFigures = 1;
elseif strcmpi(saveFiguresInput, 'N')
    saveFigures = 0;
else
    error('Invalid input. Please enter Y or N.');
end

if saveFigures
    delete(fullfile(out_dir, '*.png'));
    exportgraphics(fig1, sprintf('%s/squiggly_function.png', out_dir), 'Resolution', 300);
    exportgraphics(fig2, sprintf('%s/point_clouds.png', out_dir), 'Resolution', 300);
end

%% Iterative Closest Point (ICP) Algorithm

% Reference point cloud: pc
Q = pc;
% Input point cloud: pc_transformed
P = pc_transformed;

% The whole idea with the ICP algorithm is to, as the name implies, 
% iteratively transform the points in P such that they lie above their 
% corresponding points in Q

% Initialize the transformation errors
prev_transformation_error = inf;
transformation_error = mean(sqrt(sum((P - Q).^2, 2)));

% Set a maximum number of iterations in the case of no convergence
max_iterations = 50; % Maximum number of iterations
iteration = 0; % Initialize iteration counter

% Construct a k-dimensional (k-D) tree for the reference point cloud Q
Q_tree = KDTreeSearcher(Q);

% Continue to iterate while the transformation error continues to change
while (prev_transformation_error - transformation_error > eps) && ((iteration-1) < max_iterations)
    
    % For each point in P, find the nearest point in Q
    tic;                                % Vectorized brute force implementation
    D = pdist2(P, Q, 'euclidean');
    [~, nearest_idx] = min(D, [], 2);
    Q_nearest = Q(nearest_idx, :);
    time_b = toc;
    tic;                                % MATLAB's built-in k-D tree implementation
    nearest_idx = knnsearch(Q_tree, P);
    Q_nearest = Q(nearest_idx, :);
    time_k = toc;
    fprintf("MATLAB's k-D tree implementation was %.2f%% faster.\n", 100 * (time_b - time_k) / time_b);

    % Visualize point clouds
    figure; hold on;
    
    % Reference point cloud
    scatter(Q(:, 1), Q(:, 2), 10, 'filled', 'MarkerFaceColor', repmat(0.5, [1 3]));
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 0.1, 'Color', repmat(0.8, [1 3]));
    plot(cnrs(:, 1), cnrs(:, 2), 'LineWidth', 0.1, 'Color', 'r');

    % Input point cloud
    scatter(P(:, 1), P(:, 2), 10, 'filled', 'MarkerFaceColor', [0.5 0.5 0.5]);
    plot(xy_transformed(:, 1), xy_transformed(:, 2), 'LineWidth', 0.1, 'Color', repmat(0.8, [1 3]));
    plot(cnrs_transformed(:, 1), cnrs_transformed(:, 2), 'LineWidth', 0.1, 'Color', 'r');

    % For each point in P, draw a line to the nearest point in Q
    for i = 1:size(P, 1)
        plot([P(i, 1) Q_nearest(i, 1)], [P(i, 2) Q_nearest(i, 2)], 'LineWidth', 0.1);
    end
    
    % Get current axis limits
    x_limits = xlim;
    y_limits = ylim;
    
    % Increase limits by 20%
    x_range_increase = 0.05 * (x_limits(2) - x_limits(1));
    y_range_increase = 0.05 * (y_limits(2) - y_limits(1));
    
    % Set new axis limits
    xlim([x_limits(1) - x_range_increase, x_limits(2) + x_range_increase]);
    ylim([y_limits(1) - y_range_increase, y_limits(2) + y_range_increase]);
    
    axis equal;
    grid on; grid minor;
    title(sprintf("ICP Iteration %d | Transformation Error: %.2f", iteration, transformation_error));
    if saveFigures
        exportgraphics(gcf, sprintf('%s/ICP_iteration_%d.png', out_dir, iteration), 'Resolution', 300);
    end

    % Compute centroids of P and its nearest points in Q
    P_centroid = mean(P, 1);
    Q_nearest_centroid = mean(Q_nearest, 1);
    
    % Center P and its nearest points in Q
    P_centered = P - P_centroid;
    Q_nearest_centered = Q - Q_nearest_centroid;
    
    % Evaluate the covariance matrix H
    H = P_centered' * Q_nearest_centered;

    % Compute the rotation that maximized alignment between centered point sets
    cos_theta = (H(1, 1) + H(2, 2))/(sqrt((H(1, 1) + H(2, 2))^2 + (H(1, 2) - H(2, 1))^2));
    sin_theta = (H(1, 2) - H(2, 1))/(sqrt((H(1, 1) + H(2, 2))^2 + (H(1, 2) - H(2, 1))^2));

    % Construct the 2D rotation matrix
    R = [cos_theta, -sin_theta; ...
         sin_theta, cos_theta];
    
    % Translation vector
    t = (Q_nearest_centroid' - R * P_centroid')';
    
    % Transform P
    P = (R * P')' + t;
    xy_transformed = (R * xy_transformed')' + t;
    cnrs_transformed = (R * cnrs_transformed')' + t;

    % Update the previous transformation error
    prev_transformation_error = transformation_error;

    % Calculate the transformation error as the root mean squared distance
    transformation_error = mean(sqrt(sum((P - Q_nearest).^2, 2)));

    % Increment the iteration counter
    iteration = iteration + 1;
end

if iteration >= max_iterations
    fprintf("Maximum iterations (%d) reached - STOPPED\n", max_iterations);
else
    fprintf("CONVERGED after %d iterations\n", iteration-1);
end