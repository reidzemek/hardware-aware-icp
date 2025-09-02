function count = countLeafNodes(tree)
    if isempty(tree)
        count = 0;
    elseif isempty(tree.left) && isempty(tree.right)
        % No children — this is a leaf
        count = 1;
    else
        % Recurse on both subtrees
        count = countLeafNodes(tree.left) + countLeafNodes(tree.right);
    end
end