function fig = visualizeKDTree(tree, bounds)
    fig = figure; hold on;
    axis on; axis tight;
    title('k-dimensional Tree Visualisation');

    % Extend the range of bounds by 10%
    bounds = bounds + [-0.1 0.1 -0.1 0.1] .* (bounds(2) - bounds(1)); % Extend both min and max for x and y

    % Draw the points and their corresponding vertical/horizontal split
    drawPoint(tree, bounds);

    % Configure bounds
    xlim([bounds(1) bounds(2)]);
    ylim([bounds(3) bounds(4)]);
end
