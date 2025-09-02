function fig = drawKDTreeDiagram(tree)
    fig = figure; hold on;
    axis off; axis tight;
    title('k-dimensional (k-D) Tree Structure');

    % Draw the k-D tree nodes and edges
    drawNode(tree, 0, 0, countLeafNodes(tree), 0);

    % Get current axis limits
    x_limits = xlim;
    y_limits = ylim;
    z_limits = zlim;
    
    % Increase limits by 20%
    x_range_increase = 0.05 * (x_limits(2) - x_limits(1));
    y_range_increase = 0.05 * (y_limits(2) - y_limits(1));
    z_range_increase = 0.05 * (z_limits(2) - z_limits(1));
    
    % Set new axis limits
    xlim([x_limits(1) - x_range_increase, x_limits(2) + x_range_increase]);
    ylim([y_limits(1) - y_range_increase, y_limits(2) + y_range_increase]);
    zlim([z_limits(1) - z_range_increase, z_limits(2) + z_range_increase]);
end