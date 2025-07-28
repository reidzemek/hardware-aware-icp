function [nearest_point, nearest_distance] = KDNearest(current_node, new_point, nearest_point, nearest_distance)
    % Skip empty leaves
    if isempty(current_node)
        return;
    end

    % Update nearest point if the point for the current node is closer to the new point than the best found so far
    distance = norm(new_point - current_node.point);
    if distance < nearest_distance
        nearest_distance = distance;
        nearest_point = current_node.point;
    end

    % Perpendicular distance from the new point to the splitting axis for the current node
    split_distance = new_point(current_node.axis) - current_node.point(current_node.axis);

    % Choose near and far branches based on which side of the splitting plane the new point lies
    if split_distance < 0
        near = current_node.left; % same side as new point
        far = current_node.right; % opposite side to new point
    else
        near = current_node.right; % same side as new point
        far = current_node.left; % opposite side to new point
    end
    
    % Recursively search the near branch
    [nearest_point, nearest_distance] = KDNearest(near, new_point, nearest_point, nearest_distance);

    % Recursively search the far branch if necessary
    if abs(split_distance) < nearest_distance
        [nearest_point, nearest_distance] = KDNearest(far, new_point, nearest_point, nearest_distance);
    end
end