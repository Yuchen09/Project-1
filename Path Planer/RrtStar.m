% @file RrtStar.m
% @brief Implementation of the RRT* algorithm as a subclass of BaseClass.

%{
% @brief RrtStar - A subclass implementing the RRT* algorithm for trajectory planning.
% It provides methods to perform the RRT* algorithm, check its performance, and store the results.
%}

classdef RrtStar < BaseClass
    properties
        setMaps      %< Maps instance for the environment
        mapNumber    %< Number of the map
        mainInstance %< Reference to the Main instance
    end

    methods
        %{
        % @brief Constructor for the RrtStar class.
        % @param num The map number to use for the RRT* algorithm.
        % @param mainInstance Reference to the Main instance for data recording.
        %}
        function obj = RrtStar(num, mainInstance)
            obj.mapNumber = num;
            obj.mainInstance = mainInstance; % Store the Main instance

            % Initialization parameters
            obj.resolution = 1;
            obj.space = [0, 30, 0, 30];
            obj.startPoint = [2, 2]; 
            obj.targetPoint = [26, 26]; 
            obj.distance = 2; 
            obj.radius = 2;
            obj.treeNew = obj.startPoint';
            obj.treeOld = obj.startPoint';
            obj.nodeNumber = 1; 
            obj.targetDistance = norm(obj.targetPoint - obj.startPoint);
            obj.setMaps = Maps(num);
            obj.setObstacles(obj.setMaps.space, obj.setMaps.blocks, obj.setMaps.walls); 
        end

        %{
        % @brief Perform the performance checking of the RRT* algorithm.
        %}
        function performanceChecking(obj)
            obj.iteration = 0;
            obj.rrtStarPath = 0;
            obj.node = 0;

            tic; % time starts

            while (obj.targetDistance > obj.radius)
                obj.iteration = obj.iteration + 1;

                % Generate random point within the defined space
                [randomX, randomY] = obj.generateRandomPoint();

                % Find the nearest node and angle to the random point
                [angle, minNode] = obj.findNode(randomX, randomY);      

                % Calculate new point in the tree
                newPoint = obj.generateNewNode(minNode, angle);

                % Check if new node is valid
                if obj.isValidNode(newPoint(1), newPoint(2)) && ...
                   obj.checkSegmentIntersection(obj.treeNew(:, minNode), newPoint)

                    % Ensure treeCost is properly initialized
                    if numel(obj.treeCost) < minNode || isempty(obj.treeCost(minNode))
                        obj.treeCost(minNode) = 0; % Initialize if not already
                    end

                    % Calculate cost to new node
                    newCost = obj.treeCost(minNode) + norm(obj.treeNew(:, minNode) - newPoint);

                    % Rewire: check for better connections
                    neighbors = obj.findNeighbors(newPoint, 2 * obj.distance);
                    [minNode, newCost] = obj.rewireTree(newPoint, neighbors, minNode, newCost);

                    % Update distance to target
                    obj.targetDistance = norm(obj.targetPoint - newPoint');

                    % Store new node in the tree
                    obj.storeNewNode(minNode, newPoint, newCost);

                    % Rewiring step
                    obj.rewireNeighbors(newPoint, neighbors, newCost);
                end  
            end

            obj.drawPath();

            elapsedTime = toc; % time ends

            % Store the final iteration count, nodes count, nodes in path,   
            % total length of path, and time
            data = [obj.iteration, size(obj.treeNew, 2), obj.node, obj.rrtStarPath, elapsedTime];

            % Use the instance of Main to call recordData
            obj.mainInstance.recordData(0, data);
        end
    end
end
