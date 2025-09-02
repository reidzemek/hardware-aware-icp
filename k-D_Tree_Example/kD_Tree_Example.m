%% k-Dimensional Tree Example

out_dir_LIGHT = 'k-D_Tree_Figures_LIGHT';
out_dir_DARK = 'k-D_Tree_Figures_DARK';

% Default colors
RGB = orderedcolors("gem");

%% Generate a random set of points

% Maximum value of for a single point coordinate
range = 100;

% Total number of points (must be ≤ range)
N = 10;

% Generate unique x and y values
xVals = randperm(range, N);
yVals = randperm(range, N);

% Shuffle y to randomize the (x,y) pairs
yVals = yVals(randperm(N));

% Combine into point coordinates
points = [xVals(:), yVals(:)];

%% Build and visualize/draw the k-D tree

tree = buildKDTree(points);

fig1 = visualizeKDTree(tree, [0 range 0 range]);
fig2 = drawKDTreeDiagram(tree);

%% Add a random point to the tree visualisation

new_point = randi(100, 1, 2);

fig3 = visualizeKDTree(tree, [0 range 0 range]);

% Draw the current point
plot(new_point(1), new_point(2), 'o', 'MarkerEdgeColor', 'none', 'MarkerFaceColor', RGB(5, :), 'MarkerSize', 4);

% Add label next to the point
label = sprintf('(%g, %g)', new_point(1), new_point(2));
text(new_point(1) + 0.8, new_point(2) + 2, label, 'FontSize', 8, 'Color', RGB(5, :));

%% Find the nearest point

[nearest_point, ~] = KDNearest(tree, new_point, [], inf);

fprintf("The nearest point is (%d, %d)\n", nearest_point(1), nearest_point(2));

% Display the nearest point on the plot
figure(fig3)
plot(nearest_point(1), nearest_point(2), 'o', 'MarkerEdgeColor', 'none', 'MarkerFaceColor', RGB(4, :), 'MarkerSize', 4);
nearestLabel = sprintf('(%g, %g)', nearest_point(1), nearest_point(2));
text(nearest_point(1) + 0.8, nearest_point(2) + 2, nearestLabel, 'FontSize', 8, 'Color', RGB(4, :));

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
    fig1.Color = [0.95 0.95 0.95]; % figure background
    exportgraphics(fig1, sprintf('%s/k-D_Tree_Visualization.png', out_dir_LIGHT), 'Resolution', 300, 'Padding', 'figure');
    fig1.Theme = 'dark';
    fig1.Color = [0.15 0.15 0.15]; % figure background
    exportgraphics(fig1, sprintf('%s/k-D_Tree_Visualization.png', out_dir_DARK), 'Resolution', 300, 'Padding', 'figure');

    % Export light and dark theme versions
    fig2.Theme = 'light';
    fig2.Color = [0.95 0.95 0.95]; % figure background
    exportgraphics(fig2, sprintf('%s/k-D_Tree_Structure.png', out_dir_LIGHT), 'Resolution', 300, 'Padding', 'figure');
    fig2.Theme = 'dark';
    fig2.Color = [0.15 0.15 0.15]; % figure background
    exportgraphics(fig2, sprintf('%s/k-D_Tree_Structure.png', out_dir_DARK), 'Resolution', 300, 'Padding', 'figure');

    % Export light and dark theme versions
    fig3.Theme = 'light';
    fig3.Color = [0.95 0.95 0.95]; % figure background
    exportgraphics(fig3, sprintf('%s/k-D_Tree_Visualization_NPT.png', out_dir_LIGHT), 'Resolution', 300, 'Padding', 'figure');
    fig3.Theme = 'dark';
    fig3.Color = [0.15 0.15 0.15]; % figure background
    exportgraphics(fig3, sprintf('%s/k-D_Tree_Visualization_NPT.png', out_dir_DARK), 'Resolution', 300, 'Padding', 'figure');
end