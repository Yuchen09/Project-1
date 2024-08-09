% @file BaseClass.m
% @brief Define some general equations for RRT, RRT* and PRM

%{
% @brief BaseClass - A base class for trajectory planning (RRT,RRT*, PRM)
% It provides properties and methods for (RRT,RRT*, PRM) to do the performance checking
%}

classdef BaseClass < handle
    properties
        resolution;   %< 2D space resolution
        space;        %< Space range [x_min, x_max, y_min, y_max]
        startPoint;   %< Starting point [Sx, Sy]
        targetPoint;  %< Target point [Tx, Ty]
        distance;     %< Distance between new node and the existing nodes
        radius;       %< If the node reaches this distance, it is considered the node has reached the target point，or radius to connect neighbors for PRM
        numSample;    %< Number of samples for PRM
        walls;        %< Wall obstacles
        blocks;       %< Block obstacles
        treeNew;      %< Current node in the tree
        treeOld;      %< Parent node of the current node in the tree
        nodeNumber;   %< Node counter
        targetDistance; %< Distance to the target
        treeDistance; %< Tree distance (not used in the methods)
        treeCost;     %< Tree cost (not used in the methods)
        iteration;    %< RRT,RRT*,PRM algorithm iteration counter
        rrtPath;      %< RRT Path length counter 
        rrtStarPath;  %< RRT* Path length counter 
        node;         %< Nodes in a feasible path 
    end

    methods       
        %{
        % @brief Constructor for the BaseClass class. 
        %}
        function obj = BaseClass()
        end 

        %{
        % @brief Initialize the map with obstacles.(For RRT & RRT* & PRM)
        % @param space The boundary of the space as [x_min, x_max, y_min, y_max].
        % @param blocks The block obstacles within the space.
        % @param walls The wall obstacles within the space.
        %}
        function setObstacles(obj, space, blocks, walls)
            obj.space = space;
            obj.blocks = blocks;
            obj.walls  = walls;   
        end

        %{
        % @brief Draw the path from the start to the target node.(For RRT & RRT*)
        % @return obj.node  Number of Nodes in a feasible path 
        % @return obj.rrtStarPath  RRT* Path length
        % @return  obj.rrtPath     RRT Path length(needs to consider property distance)
        %}
        function drawPath(obj)
            currentIndex = length(obj.treeNew);

            while currentIndex ~= 1
                % Node number in path
                obj.node = obj.node + 1;

                % RRT* path length
                obj.rrtStarPath = obj.rrtStarPath + norm(obj.treeNew(:, currentIndex) - obj.treeOld(:, currentIndex));

                % RRT path length
                obj.rrtPath = obj.rrtPath + 1;

                for i = 1:length(obj.treeNew)
                    if obj.treeNew(1, i) == obj.treeOld(1, currentIndex) && ...
                       obj.treeNew(2, i) == obj.treeOld(2, currentIndex)
                        currentIndex = i;
                        break;
                    end
                end
            end
        end

        %{
        % @brief Generate a random point within the defined space.(For RRT & RRT*)
        % @return randomX X coordinate of the random point.
        % @return randomX Y coordinate of the random point.
        %}
        function [randomX, randomY] = generateRandomPoint(obj)
            randomX = (obj.space(2) - obj.space(1)) * rand() + obj.space(1);
            randomY = (obj.space(4) - obj.space(3)) * rand() + obj.space(3);
        end

        %{
        % @brief Generate a new node based on the nearest node and angle.(For RRT* only)
        % @param minNode Index of the nearest node.
        % @param angle Angle towards the random point.
        % @return newPoint The coordinates of the new node.
        %}
        function newPoint = generateNewNode(obj, minNode, angle)
            newX = obj.treeNew(1, minNode) + obj.distance * cos(angle);
            newY = obj.treeNew(2, minNode) + obj.distance * sin(angle);
            newPoint = [newX; newY];
        end

        %{
        % @brief Check if the node is within the valid space.(For RRT & RRT*)
        % @param x X coordinate of the node.
        % @param y Y coordinate of the node.
        % @return isValid True if the node is within the space, otherwise false.
        %}
        function isValid = isValidNode(obj, x, y)
            isValid = (x > obj.space(1) + 1 && x < obj.space(2) - 1 && ...
                       y > obj.space(3) + 1 && y < obj.space(4) - 1);
        end

        %{
        % @brief Check if a node is inside any obstacle (for PRM only).
        % @param point The coordinates of the node to check.
        % @return flag True if the node is inside an obstacle, otherwise false.
        %}
        function flag = checkCollision(obj, point)
            flag = false;
            for i = 1:size(obj.blocks, 1)
                if inpolygon(point(1), point(2), obj.blocks(i, 1:4), obj.blocks(i, 5:8))
                    flag = true;
                    return;
                end
            end
        end

        %{
        % @brief Check if the segment intersects any obstacle (for RRT and RRT*).
        % @param startPoint The starting point of the segment.
        % @param end_point The ending point of the segment.
        % @return flag True if the segment does not intersect any obstacle, otherwise false.
        %}
        function flag = checkSegmentIntersection(obj, startPoint, end_point)
            flag = 1;
            num_points = 10;
            for t = linspace(0, 1, num_points)
                x = startPoint(1) + t * (end_point(1) - startPoint(1));
                y = startPoint(2) + t * (end_point(2) - startPoint(2));
                for i = 1:size(obj.blocks, 1)
                    if inpolygon(x, y, obj.blocks(i, 1:4), obj.blocks(i, 5:8))
                        flag = 0;
                        return;
                    end
                end
            end
        end

        %{
        % @brief Rewire the tree by checking for better connections (for RRT* only).
        % @param newPoint The new node to consider.
        % @param neighbors The neighboring nodes to check.
        % @param minNode The current best node for connection.
        % @param newCost The current best cost for connection.
        % @return minNode The index of the new best node.
        % @return newCost The new best cost.
        %}
        function [minNode, newCost] = rewireTree(obj, newPoint, neighbors, minNode, newCost)
            for i = neighbors
                if obj.checkSegmentIntersection(obj.treeNew(:, i), newPoint)
                    cost_to_i = obj.treeCost(i) + norm(obj.treeNew(:, i) - newPoint);
                    if cost_to_i < newCost
                        newCost = cost_to_i;
                        minNode = i;
                    end
                end
            end
        end

        %{
        % @brief Store the new node in the tree (for RRT* only).
        % @param minNode The index of the parent node.
        % @param newPoint The coordinates of the new node.
        % @param newCost The cost of reaching the new node.
        %}
        function storeNewNode(obj, minNode, newPoint, newCost)
            obj.nodeNumber = obj.nodeNumber + 1;
            obj.treeNew(:, obj.nodeNumber) = newPoint;
            obj.treeOld(:, obj.nodeNumber) = obj.treeNew(:, minNode);
            obj.treeCost(obj.nodeNumber) = newCost;
        end

        %{
        % @brief Rewire neighbors to the new node if it offers a better path (for RRT* only).
        % @param newPoint The coordinates of the new node.
        % @param neighbors The neighboring nodes to check.
        % @param newCost The cost of reaching the new node.
        %}
        function rewireNeighbors(obj, newPoint, neighbors, newCost)
            for i = neighbors
                if obj.checkSegmentIntersection(obj.treeNew(:, i), newPoint)
                    costFromNew = newCost + norm(obj.treeNew(:, i) - newPoint);
                    if costFromNew < obj.treeCost(i)
                        obj.treeCost(i) = costFromNew;
                        obj.treeOld(:, i) = newPoint;
                    end
                end
            end
        end

        %{
        % @brief Find the closest node in the tree to a given random point (for RRT & RRT*)
        % @param randomX X coordinate of the random point.
        % @param randomY Y coordinate of the random point.
        % @return angle The angle from the nearest node to the random point.
        % @return min_node The index of the nearest node.
        %}
        function [angle, min_node] = findNode(obj, random_x, random_y)
            distance = sqrt((random_x - obj.treeNew(1, :)).^2 + (random_y - obj.treeNew(2, :)).^2);
            [~, min_node] = min(distance);
            angle = atan2(random_y - obj.treeNew(2, min_node), random_x - obj.treeNew(1, min_node));
        end

        %{
        % @brief Perform the A* algorithm for finding the shortest path (for PRM only).
        % @param adj_matrix The adjacency matrix representing the graph.
        % @param nodes The coordinates of the nodes in the graph.
        % @param start_idx The index of the start node.
        % @param target_idx The index of the target node.
        % @return path The sequence of nodes forming the shortest path.
        % @return cost The total cost of the path.
        %}
        function [path, cost] = astar(obj, adj_matrix, nodes, start_idx, target_idx)
            num_nodes = size(nodes, 1);
            open_set = start_idx;
            came_from = nan(1, num_nodes);

            g_score = inf(1, num_nodes);
            g_score(start_idx) = 0;

            f_score = inf(1, num_nodes);
            f_score(start_idx) = obj.heuristic_cost_estimate(nodes(start_idx, :), nodes(target_idx, :));

            while ~isempty(open_set)
                obj.AStarIteration = obj.AStarIteration + 1;
                [~, idx] = min(f_score(open_set));
                current = open_set(idx);

                if current == target_idx
                    path = obj.reconstruct_path(came_from, current);
                    cost = g_score(target_idx);
                    return;
                end

                open_set(idx) = [];

                neighbors = find(adj_matrix(current, :));
                for neighbor = neighbors
                    tentative_g_score = g_score(current) + norm(nodes(current, :) - nodes(neighbor, :));
                    if tentative_g_score < g_score(neighbor)
                        came_from(neighbor) = current;
                        g_score(neighbor) = tentative_g_score;
                        f_score(neighbor) = g_score(neighbor) + obj.heuristic_cost_estimate(nodes(neighbor, :), nodes(target_idx, :));

                        if ~ismember(neighbor, open_set)
                            open_set = [open_set, neighbor];
                        end
                    end
                end
            end

            path = [];
            cost = inf;
        end

        %{
        % @brief Heuristic cost estimate for A* algorithm (for PRM only).
        % @param node The current node coordinates.
        % @param goal The goal node coordinates.
        % @return cost The heuristic cost (Euclidean distance).
        %}
        function cost = heuristic_cost_estimate(~, node, goal)
            cost = norm(node - goal);
        end

        %{
        % @brief Reconstruct the path from the A* algorithm (for PRM only).
        % @param came_from Array indicating the predecessor of each node.
        % @param current The current node index.
        % @return path The sequence of nodes forming the reconstructed path.
        %}
        function path = reconstruct_path(~, came_from, current)
            total_path = current;
            while ~isnan(came_from(current))
                current = came_from(current);
                total_path = [current, total_path];
            end
            path = total_path;
        end

        %{
        % @brief Find neighbors within a given radius (for RRT* only).
        % @param new The coordinates of the new node.
        % @param radius The radius to consider for neighbors.
        % @return neighbors The indices of the neighboring nodes.
        %}
        function neighbors = findNeighbors(obj, new, radius)
            neighbors = [];
            for i = 1:size(obj.treeNew, 2)
                if norm(obj.treeNew(:, i) - new) < radius
                    neighbors = [neighbors, i];
                end
            end
        end
        
    end
end
