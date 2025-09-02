function drawNode(node, x, y, dx, depth)
    % Skip empty leaves
    if isempty(node)
        return;
    end

    % Current node label
    label = sprintf('(%g, %g)', node.point(1), node.point(2));

    % Draw the current node
    h = text(x, y, label, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'BackgroundColor', repmat(0.7, 1, 3), 'Color', 'black');

    % Children
    if ~isempty(node.left)
        xL = x - dx;
        yL = y - 1;
        line([x, xL], [y, yL], 'Color', repmat(0.7, 1, 3));
        drawNode(node.left, xL, yL, dx / 2, depth + 1);
    end
    if ~isempty(node.right)
        xR = x + dx;
        yR = y - 1;
        line([x, xR], [y, yL], 'Color', repmat(0.7, 1, 3));
        drawNode(node.right, xR, yR, dx / 2, depth + 1);
    end

    % Bring label to front
    uistack(h, 'top');
end