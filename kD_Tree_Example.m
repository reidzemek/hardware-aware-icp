%% k-Dimensional Tree Example

out_dir = 'k-D_Tree_Example';

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
plot(new_point(1), new_point(2), 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4);

% Add label next to the point
label = sprintf('(%g, %g)', new_point(1), new_point(2));
text(new_point(1) + 0.8, new_point(2) + 2, label, 'FontSize', 8, 'Color', 'r');

%% Find the nearest point

[nearest_point, ~] = KDNearest(tree, new_point, [], inf);

fprintf("The nearest point is (%d, %d)\n", nearest_point(1), nearest_point(2));

% Display the nearest point on the plot
figure(fig3)
plot(nearest_point(1), nearest_point(2), 'bo', 'MarkerFaceColor', 'g', 'MarkerSize', 4);
nearestLabel = sprintf('(%g, %g)', nearest_point(1), nearest_point(2));
text(nearest_point(1) + 0.8, nearest_point(2) + 2, nearestLabel, 'FontSize', 8, 'Color', 'g');

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
    exportgraphics(fig1, sprintf('%s/k-D_Tree_Visualization.png', out_dir), 'Resolution', 300);
    exportgraphics(fig2, sprintf('%s/k-D_Tree_Structure.png', out_dir), 'Resolution', 300);
    exportgraphics(fig3, sprintf('%s/k-D_Tree_Visualization_NPT.png', out_dir), 'Resolution', 300);
end