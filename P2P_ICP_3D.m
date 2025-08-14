%% Point-to-point Iterative Closest Point Algorithm - 3D

% 3-dimensional hardware friendly implementation of the point-to-point
% Iterative Closest Point (ICP) point cloud registration algorithm

out_dir = 'P2P_ICP_3D';

%% Generate an irregular smooth squiggly surface

% Parameters
range = 100;            % size of 3D cubic volume containing surface
extent = 1;             % factor to scale range and define the horizontal extent of the surface
n_points = 10;          % number of horizontal points per range increment
downsample_factor = 4;  % must be a factor of range/extent

% Check that downsample_factor is a factor of range/extent
if mod(range / extent, downsample_factor) ~= 0
    error('downsample_factor must be a factor of range/extent');
end

% Adjust downsample factor based on extent
downsample_factor = downsample_factor * extent;

% Initialize time signal vector for horizontal axes (x and y)
t = linspace(0, extent*range, (range+1)*n_points);
x = zeros(size(t));
y = zeros(size(t));

% Number of harmonic components for horizontal signal vectors (more terms = more irregularity)
num_terms = 10;

% Stocastically generate both horizontal signals
for k = 1:num_terms
    A1 = ((rand()*(sqrt(range)/3))+(2*(sqrt(range)/3)))*(1/k);
    f1 = (rand()*1/range)+1/(2*range);
    phi1 = rand()*2*pi;
    if rand() > 0.5
        x = x + A1 * sin(2*pi*f1*t + phi1);
    else
        x = x + A1 * cos(2*pi*f1*t + phi1);
    end
    
    A2 = ((rand()*(sqrt(range)/3))+(2*(sqrt(range)/3)))*(1/k);
    f2 = (rand()*1/range)+1/(2*range);
    phi2 = rand()*2*pi;
    if rand() > 0.5
        y = y + A2 * sin(2*pi*f2*t + phi2);
    else
        y = y + A2 * cos(2*pi*f2*t + phi2);
    end
end

% Create a surface grid and multiply horizontal signal vectors to define the surface along the vertical axis
[X, Y] = meshgrid(t, t);
Z = (x' * y)/2;

% Densely sampled 3D point cloud
XYZ = [X(:) Y(:) Z(:)];

%% Dowsampled 2D Point Cloud

% Downsample and add x and y offsets
offset = rand(1, 2)*range-(range/2);
XYZ(:, 1:2) = XYZ(:, 1:2) + offset;
pc = zeros((n_points*downsample_factor*extent+1)^2, 3);
temp_X = reshape(XYZ(:, 1), repmat((range+1)*n_points*extent, [1, 2]));
pc(:, 1) = reshape(temp_X(1:range/downsample_factor:end, 1:range/downsample_factor:end), [], 1);
temp_Y = reshape(XYZ(:, 2), repmat((range+1)*n_points*extent, [1, 2]));
pc(:, 2) = reshape(temp_Y(1:range/downsample_factor:end, 1:range/downsample_factor:end), [], 1);
temp_Z = reshape(XYZ(:, 3), repmat((range+1)*n_points*extent, [1, 2]));
pc(:, 3) = reshape(temp_Z(1:range/downsample_factor:end, 1:range/downsample_factor:end), [], 1);

% Define the corners for a bounding cube around pc with a 10% border
min_X = min(pc(:, 1));
max_X = max(pc(:, 1));
min_Y = min(pc(:, 2));
max_Y = max(pc(:, 2));
min_Z = min(pc(:, 3));
max_Z = max(pc(:, 3));
border = min([0.1*(max_X-min_X), 0.1*(max_Y-min_Y), 0.1*(max_Z-min_Z)]);
cnrs = [min_X-border, min_Y-border, min_Z-border; ...
        max_X+border, min_Y-border, min_Z-border; ...
        max_X+border, max_Y+border, min_Z-border; ...
        min_X-border, max_Y+border, min_Z-border; ...
        min_X-border, min_Y-border, max_Z+border; ...
        max_X+border, min_Y-border, max_Z+border; ...
        max_X+border, max_Y+border, max_Z+border; ...
        min_X-border, max_Y+border, max_Z+border; ...
        min_X-border, min_Y-border, min_Z-border; ...
        max_X+border, min_Y-border, min_Z-border; ...
        max_X+border, min_Y-border, max_Z+border; ...
        min_X-border, min_Y-border, max_Z+border; ...
        min_X-border, max_Y+border, min_Z-border; ...
        max_X+border, max_Y+border, min_Z-border; ...
        max_X+border, max_Y+border, max_Z+border; ...
        min_X-border, max_Y+border, max_Z+border; ...
        min_X-border, min_Y-border, min_Z-border; ...
        min_X-border, max_Y+border, min_Z-border; ...
        min_X-border, max_Y+border, max_Z+border; ...
        min_X-border, min_Y-border, max_Z+border; ...
        max_X+border, min_Y-border, min_Z-border; ...
        max_X+border, max_Y+border, min_Z-border; ...
        max_X+border, max_Y+border, max_Z+border; ...
        max_X+border, min_Y-border, max_Z+border];

%% Transform the point cloud and add some noise

% Random rotation andles for each axis (x, y & z)
angles = rand(3, 1) * 2 * pi;

% Per-axis rotation matrices
Rx = [1, 0, 0;
      0, cos(angles(1)), -sin(angles(1));
      0, sin(angles(1)), cos(angles(1))];

Ry = [cos(angles(2)), 0, sin(angles(2));
      0, 1, 0;
      -sin(angles(2)), 0, cos(angles(2))];

Rz = [cos(angles(3)), -sin(angles(3)), 0;
      sin(angles(3)), cos(angles(3)), 0;
      0, 0, 1];

% 3-dimensioinal rotation matrix (axis rotations applied in ZYX order)
R = Rz * Ry * Rx;

% Random translation vector
t = rand(3, 1)*2*range;  % Random translation in the range [0, 10]

% Add Gaussian noise to the point cloud
noise_level = range/200; % Adjust the noise level as needed
noise = noise_level * randn(size(pc)); % Generate Gaussian noise
pc_noisy = pc + noise; % Add noise to the point cloud

% Apply transformation: rotate then translate
pc_transformed = ((R * pc_noisy') + t)';
XYZ_transformed = ((R * XYZ') + t)';
cnrs_transformed = ((R * cnrs') + t)';

%% Visualize the point clouds

% Visualize the surface including the downsampled point cloud
figure; hold on;
surf(reshape(XYZ(:, 1), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ(:, 2), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ(:, 3), repmat((range+1)*n_points*extent, [1, 2])), ...
     'EdgeColor', 'none');
scatter3(reshape(pc(:, 1), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc(:, 2), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc(:, 3), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         'filled', 'MarkerFaceColor', 'r', 'SizeData', 5);
hold off; axis equal; view(3);
grid on; grid minor
colormap(repmat(linspace(0.2, 0.8, 256)', [1, 3]));

xlim([min(XYZ(:, 1)) max(XYZ(:, 1))]);
ylim([min(XYZ(:, 2)) max(XYZ(:, 2))]);

xlabel('x');
ylabel('y');
zlabel('z');
title('Irregular Smooth Squiggly Surface');
fig1 = gcf;

% Visualize point clouds before ICP
figure; hold on;

% Original point cloud
scatter3(reshape(pc(:, 1), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc(:, 2), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc(:, 3), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         'filled', 'MarkerFaceColor', 'r', 'SizeData', 5);
surf(reshape(XYZ(:, 1), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ(:, 2), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ(:, 3), repmat((range+1)*n_points*extent, [1, 2])), ...
     'EdgeColor', 'none');
for i = 1:4:24
    fill3(cnrs(i:(i+3), 1), cnrs(i:(i+3), 2), cnrs(i:(i+3), 3), ...
          [0.6 0.8 0.9], 'FaceAlpha', 0.3, 'EdgeColor', [0.64 0.82 0.91]);
end

% Transformed point cloud
scatter3(reshape(pc_transformed(:, 1), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc_transformed(:, 2), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         reshape(pc_transformed(:, 3), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
         'filled', 'MarkerFaceColor', 'r', 'SizeData', 5);
surf(reshape(XYZ_transformed(:, 1), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ_transformed(:, 2), repmat((range+1)*n_points*extent, [1, 2])), ...
     reshape(XYZ_transformed(:, 3), repmat((range+1)*n_points*extent, [1, 2])), ...
     'EdgeColor', 'none');
for i = 1:4:24
    fill3(cnrs_transformed(i:(i+3), 1), cnrs_transformed(i:(i+3), 2), cnrs_transformed(i:(i+3), 3), ...
          [0.6 0.8 0.9], 'FaceAlpha', 0.3, 'EdgeColor', [0.64 0.82 0.91]);
end

hold off; axis equal; view(3);
grid on; grid minor
colormap(repmat(linspace(0.2, 0.8, 256)', [1, 3]));

% Minimum axis limits
x_limits = [min([cnrs(:, 1); cnrs_transformed(:, 1)]), max([cnrs(:, 1); cnrs_transformed(:, 1)])];
y_limits = [min([cnrs(:, 2); cnrs_transformed(:, 2)]), max([cnrs(:, 2); cnrs_transformed(:, 2)])];
z_limits = [min([cnrs(:, 3); cnrs_transformed(:, 3)]), max([cnrs(:, 3); cnrs_transformed(:, 3)])];

% Increase limits by 20%
x_range_increase = 0.1 * (x_limits(2) - x_limits(1));
y_range_increase = 0.1 * (y_limits(2) - y_limits(1));

% Set new axis limits
xlim([x_limits(1) - x_range_increase, x_limits(2) + x_range_increase]);
ylim([y_limits(1) - y_range_increase, y_limits(2) + y_range_increase]);

xlabel('x');
ylabel('y');
zlabel('z');
title(sprintf('Rotation: %.2f°(X) %.2f°(Y) %.2f°(Z), Translation: %.2f, %.2f, %.2f', rad2deg(angles(1)), rad2deg(angles(2)), rad2deg(angles(3)), t(1), t(2), t(3)));
fig2 = gcf;

%% Prompt the user to save figures

% Output directory
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
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
    delete(fullfile(out_dir, '*.png'));
    exportgraphics(fig1, sprintf('%s/squiggly_surface.png', out_dir), 'Resolution', 300);
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
max_iterations = 100; % Maximum number of iterations
iteration = 0; % Initialize iteration counter

% Construct a k-dimensional (k-D) tree for the reference point cloud Q
Q_tree = KDTreeSearcher(Q);

% Continue to iterate while the transformation error continues to change
while (prev_transformation_error - transformation_error > eps) && (iteration - 1 < max_iterations)

    % For each pooint in P, find the nearest point in Q
    nearest_idx = knnsearch(Q_tree, P);
    Q_nearest = Q(nearest_idx, :);

    % Visualize point clouds
    figure; hold on;

    % Reference point cloud
    scatter3(reshape(Q(:, 1), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             reshape(Q(:, 2), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             reshape(Q(:, 3), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             'filled', 'MarkerFaceColor', 'r', 'SizeData', 5);
    surf(reshape(XYZ(:, 1), repmat((range+1)*n_points*extent, [1, 2])), ...
         reshape(XYZ(:, 2), repmat((range+1)*n_points*extent, [1, 2])), ...
         reshape(XYZ(:, 3), repmat((range+1)*n_points*extent, [1, 2])), ...
         'EdgeColor', 'none');
    for i = 1:4:24
        fill3(cnrs(i:(i+3), 1), cnrs(i:(i+3), 2), cnrs(i:(i+3), 3), ...
              [0.6 0.8 0.9], 'FaceAlpha', 0.3, 'EdgeColor', [0.64 0.82 0.91]);
    end

    % Input point cloud
    scatter3(reshape(P(:, 1), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             reshape(P(:, 2), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             reshape(P(:, 3), repmat(n_points*downsample_factor*extent+1, [1, 2])), ...
             'filled', 'MarkerFaceColor', 'r', 'SizeData', 5);
    surf(reshape(XYZ_transformed(:, 1), repmat((range+1)*n_points*extent, [1, 2])), ...
         reshape(XYZ_transformed(:, 2), repmat((range+1)*n_points*extent, [1, 2])), ...
         reshape(XYZ_transformed(:, 3), repmat((range+1)*n_points*extent, [1, 2])), ...
         'EdgeColor', 'none');
    for i = 1:4:24
        fill3(cnrs_transformed(i:(i+3), 1), cnrs_transformed(i:(i+3), 2), cnrs_transformed(i:(i+3), 3), ...
              [0.6 0.8 0.9], 'FaceAlpha', 0.3, 'EdgeColor', [0.64 0.82 0.91]);
    end

    % For each point in P, draw a line to the nearest point in Q
    for i = 1:size(P, 1)
        plot3([P(i, 1) Q_nearest(i, 1)], [P(i, 2) Q_nearest(i, 2)], [P(i, 3) Q_nearest(i, 3)], 'LineWidth', 0.1);
    end

    hold off; axis equal; view(3);
    grid on; grid minor
    colormap(repmat(linspace(0.2, 0.8, 256)', [1, 3]));
    
    % Minimum axis limits
    x_limits = [min([cnrs(:, 1); cnrs_transformed(:, 1)]), max([cnrs(:, 1); cnrs_transformed(:, 1)])];
    y_limits = [min([cnrs(:, 2); cnrs_transformed(:, 2)]), max([cnrs(:, 2); cnrs_transformed(:, 2)])];
    z_limits = [min([cnrs(:, 3); cnrs_transformed(:, 3)]), max([cnrs(:, 3); cnrs_transformed(:, 3)])];
    
    % Increase limits by 20%
    x_range_increase = 0.1 * (x_limits(2) - x_limits(1));
    y_range_increase = 0.1 * (y_limits(2) - y_limits(1));
    
    % Set new axis limits
    xlim([x_limits(1) - x_range_increase, x_limits(2) + x_range_increase]);
    ylim([y_limits(1) - y_range_increase, y_limits(2) + y_range_increase]);
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
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

    % Extract the skew vector from the cross-covariance matrix H
    delta = [H(2, 3)-H(3, 2); ...
             H(3, 1)-H(1, 3); ...
             H(1, 2)-H(2, 1)];
    
    % Use Horn's method to construct the 4x4 quaternion characteristic matrix N
    N = zeros(4, 4);
    N(1, 1) = trace(H);
    N(1, 2:4) = delta';
    N(2:4, 1) = delta;
    N(2:4, 2:4) = [2*H(1, 1)-trace(H), H(1, 2)+H(2, 1),    H(1, 3)+H(3, 1); ...
                   H(2, 1)+H(1, 2),    2*H(2, 2)-trace(H), H(2, 3)+H(3, 2); ...
                   H(3, 1)+H(1, 3),    H(3, 2)+H(2, 3),    2*H(3, 3)-trace(H)];
    
    % Use the power iteration method to compute the dominant eigen vector, corresponding to the unit quaternion
    q = [1; 0; 0; 0];
    K = 100;
    for k = 1:K
        q_temp = N * q;
        q_temp = q_temp / norm(q_temp);
        if norm(q - q_temp) < eps
            break
        end
        q = q_temp;
    end
    
    % Quaternion to rotation matrix conversion
    R = [1 - 2*(q(3)^2 + q(4)^2),   2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); ...
         2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2),   2*(q(3)*q(4) - q(2)*q(1)); ...
         2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];

    % Translation vector
    t = (Q_nearest_centroid' - R * P_centroid')';

    % Transform P
    P = (R * P')' + t;
    XYZ_transformed = (R * XYZ_transformed')' + t;
    cnrs_transformed = (R * cnrs_transformed')' +t;

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