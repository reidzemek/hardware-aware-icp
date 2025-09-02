function kdTree = buildKDTree(points, depth)
    if nargin < 2
        depth = 0;
    end

    if isempty(points)
        kdTree = [];
        return;
    end

    % Alternate between x (dim=1) and y (dim=2)
    k = size(points, 2);
    axis = mod(depth, k) + 1;

    % Sort points by current axis
    points = sortrows(points, axis);
    medianIdx = floor(size(points, 1) / 2) + 1;

    % Create node
    kdTree.point = points(medianIdx, :);
    kdTree.axis = axis;
    kdTree.left = buildKDTree(points(1:medianIdx-1, :), depth + 1);
    kdTree.right = buildKDTree(points(medianIdx+1:end, :), depth + 1);
end