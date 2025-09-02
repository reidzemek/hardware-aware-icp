function drawPoint(tree, bounds)
    % Default colors
    RGB_DARK = orderedcolors("glow");

    % Skip empty leaves
    if isempty(tree)
        return;
    end

    % Draw the current point
    plot(tree.point(1), tree.point(2), 'o', 'MarkeredgeColor', 'none', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerSize', 4);

    % Add label next to the point
    label = sprintf('(%g, %g)', tree.point(1), tree.point(2));
    text(tree.point(1) + 0.8, tree.point(2) + 2, label, 'FontSize', 8);

    % Check if current node is a leaf node - split only on branch nodes
    if ~(isempty(tree.left) && isempty(tree.right))
        if tree.axis == 1
            % Vertical split
            line([tree.point(1), tree.point(1)], [bounds(3), bounds(4)], 'Color', RGB_DARK(1, :), 'LineStyle', '--');
            drawPoint(tree.left, [bounds(1), tree.point(1), bounds(3), bounds(4)]);
            drawPoint(tree.right, [tree.point(1), bounds(2), bounds(3), bounds(4)]);
        else
            % Horizontal split
            line([bounds(1), bounds(2)], [tree.point(2), tree.point(2)], 'Color', RGB_DARK(2, :), 'LineStyle', '--');
            drawPoint(tree.left, [bounds(1), bounds(2), bounds(3), tree.point(2)]);
            drawPoint(tree.right, [bounds(1), bounds(2), tree.point(2), bounds(4)]);
        end
    end
end