%% Point-to-point Iterative Closest Point Algorithm - 2D

% 2-dimensional hardware friendly implementation of the point-to-point 
% Iterative Closest Point (ICP) point cloud registration algorithm

out_dir_LIGHT = 'P2P_ICP_2D_Figures_LIGHT';
out_dir_DARK = 'P2P_ICP_2D_Figures_DARK';

%% Generate an irregular smooth squiggly function

% Parameters
range = 100;            % size of 2D square containing line (must be a whole number multiple of 4)
extent = 5;             % factor to scale range and define the horizontal extent of the line
n_points = 10;          % number of horizontal points per range increment
downsample_factor = 4;  % source point cloud downsample factor

% Initialize time signal vector for horizontal axis (x)
t = linspace(0, extent*range, extent*range*n_points+1);
y = zeros(size(t));

% Number of harmonic components (more terms = more irregularity)
num_terms = 100;

% Stocatically generated function
for k = 1:num_terms
    A = ((rand()*(2*(range/2)/3))+((range/2)/3))*(1/k);
    f = (rand()*1/range)+1/(2*range);
    phi = rand()*2*pi;
    if rand() > 0.5
        y = y + A * sin(2*pi*f*t + phi);
    else
        y = y + A * cos(2*pi*f*t + phi);
    end
end

% Densely sampled 2D point cloud
xy = [t' y'];

%% 2D Point Cloud

% Downsample the function to generate a 2D point cloud
pc = xy(1:range/4:end, :);

% Center the 2D point clout at zero (center the function data as well)
pc = pc - mean(pc, 1);
xy = xy - mean(xy, 1);

% Define corners for a bounding box around pc with a 20% border
border = min([0.2*(max(pc(:, 1))-min(pc(:, 1))), 0.2*(max(pc(:, 2))-min(pc(:, 2)))]);
cnrs = [min(pc(:, 1))-border, min(pc(:, 2))-border; ...
        max(pc(:, 1))+border, min(pc(:, 2))-border; ...
        max(pc(:, 1))+border, max(pc(:, 2))+border; ...
        min(pc(:, 1))-border, max(pc(:, 2))+border];
cnrs(end+1,:) = cnrs(1,:);

%% Downsample the point cloud

% Downsample the point cloud according to downsample_factor using binning
num_bins = ceil(size(pc, 1) / downsample_factor);
pc_downsampled = zeros(num_bins, 2);

% Compute the average of neighboring values and store the downsampled point cloud
for i = 1:num_bins
    bin_start = (i-1) * downsample_factor + 1;
    bin_end = min(i * downsample_factor, size(pc, 1));
    pc_downsampled(i, :) = mean(pc(bin_start:bin_end, :), 1);
end

%% Add some noise to simulate a real world acquisition

% Add Gaussian noise to the point cloud
noise_level = range/80; % Adjust the noise level as needed
noise = noise_level * randn(size(pc_downsampled)); % Generate Gaussian noise
pc_noisy = pc_downsampled + noise; % Add noise to the point cloud

%% Transform the point cloud

% Generate random rotation angle (in radians)
theta = (rand() * deg2rad(30)) - deg2rad(15);

% 2D rotation matrix
R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];

% Random translation vector (1x2)
t = rand(1, 2)*2*range-range;

% Apply transformation: rotate then translate
pc_transformed = (R * pc_noisy')' + t;
xy_transformed = (R * xy')' + t;
cnrs_transformed = (R * cnrs')' + t;

%% Visualize point clouds

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

% Output directory
if ~exist(out_dir_LIGHT, 'dir')
    mkdir(out_dir_LIGHT);
elseif ~exist(out_dir_DARK, 'dir')
    mkdir(out_dir_DARK);
end

saveFiguresInput = input('Set save figures flag? (Y/N): ', 's');

if strcmpi(saveFiguresInput, 'Y')
    saveFigures = 1;
elseif strcmpi(saveFiguresInput, 'N')
    saveFigures = 0;
else
    error('Invalid input. Please enter Y or N.');
end

if saveFigures
    delete(fullfile(out_dir_LIGHT, '*'));
    delete(fullfile(out_dir_DARK, '*'));
    
    % Export light and dark theme versions
    fig1.Theme = 'light';
    fig1.Color = [0.95, 0.95, 0.95]; % figure background
    exportgraphics(fig1, sprintf('%s/squiggly_function.png', out_dir_LIGHT), 'Resolution', 300, 'Padding', 'figure');
    fig1.Theme = 'dark';
    fig1.Color = [0.15 0.15 0.15]; % figure background
    exportgraphics(fig1, sprintf('%s/squiggly_function.png', out_dir_DARK), 'Resolution', 300, 'Padding', 'figure');

    % Export light and dark theme versions
    fig2.Theme = 'light';
    fig2.Color = [0.95, 0.95, 0.95]; % figure background
    exportgraphics(fig2, sprintf('%s/point_clouds.png', out_dir_LIGHT), 'Resolution', 300, 'Padding', 'figure');
    fig2.Theme = 'dark';
    fig2.Color = [0.15 0.15 0.15]; % figure background
    exportgraphics(fig2, sprintf('%s/point_clouds.png', out_dir_DARK), 'Resolution', 300, 'Padding', 'figure');

    % Close figures to save memory
    close(fig1);
    close(fig2);
end

%% Iterative Closest Point (ICP) Algorithm

% Reference point cloud: pc
Q = pc;
% Input point cloud: pc_transformed
P = pc_transformed;

% The whole idea with the ICP algorithm is to, as the name implies, 
% iteratively transform the points in P such that they lie above their 
% corresponding points in Q

% Construct a k-dimensional (k-D) tree for the reference point cloud Q
Q_tree = KDTreeSearcher(Q);

% For each point in P, find the nearest point in Q
nearest_idx = knnsearch(Q_tree, P);
Q_nearest = Q(nearest_idx, :);

% Initialize the previous transformation error
prev_transformation_error = inf;

% Calculate the transformation error as the root mean squared distance
transformation_error = mean(sqrt(sum((P - Q_nearest).^2, 2)));

% Set a maximum number of iterations in the case of no convergence
max_iterations = 50; % Maximum number of iterations
iteration = 0; % Initialize iteration counter

% Continue to iterate while the transformation error continues to change
while (abs(prev_transformation_error - transformation_error) > 1e-4) && ((iteration-1) < max_iterations)
    
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
        fig = gcf;

        % Export light adn dark theme versions
        fig.Theme = 'light';
        fig.Color = [0.95 0.95 0.95];
        exportgraphics(fig, sprintf('%s/ICP_iteration_%d.png', out_dir_LIGHT, iteration), 'Resolution', 300, 'Padding', 'figure');
        fig.Theme = 'dark';
        fig.Color = [0.15 0.15 0.15];
        exportgraphics(fig, sprintf('%s/ICP_iteration_%d.png', out_dir_DARK, iteration), 'Resolution', 300, 'Padding', 'figure');
        
        close(gcf);
    end

    % Compute centroids of P and its nearest points in Q
    P_centroid = mean(P, 1);
    Q_nearest_centroid = mean(Q_nearest, 1);
    
    % Center P and its nearest points in Q
    P_centered = P - P_centroid;
    Q_nearest_centered = Q_nearest - Q_nearest_centroid;
    
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