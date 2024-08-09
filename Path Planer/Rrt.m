% @file Rrt.m
% @brief Implementation of the RRT algorithm.

%{
% @brief Rrt - A subclass for implementing the RRT algorithm.
% This class extends the BaseClass and provides methods to perform the RRT algorithm.
%}

classdef Rrt < BaseClass
    properties
        setMaps       %< Maps instance to define the space, blocks, and walls
        mapNumber     %< Identifier for the map number
        mainInstance  %< Reference to the Main instance
    end

    methods
        %{
        % @brief Constructor for the Rrt class.
        % @param num The map number.
        % @param mainInstance Reference to the Main instance for data recording.
        %}
        function obj = Rrt(num, mainInstance)
            obj.mapNumber = num;
            % Store the Main instance
            obj.mainInstance = mainInstance; 

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
        % @brief Perform the RRT algorithm and check its performance.
        % This method runs the main RRT loop, generates random points,
        % checks for valid nodes, and updates the tree. 
        % It also records the performance data using the Main instance.
        %}
        function performanceChecking(obj)
            obj.iteration = 0;
            obj.rrtPath = 0;
            obj.node = 0;
           
            tic; % time starts

            % Main RRT loop
            while (obj.targetDistance > obj.radius)

                % Counting the number of iterations for RRT
                obj.iteration = obj.iteration + 1;

                % Generate random point within the defined space
                [randomX, randomY] = obj.generateRandomPoint();

                % Find the nearest node and angle to the random point
                [angle, minNode] = obj.findNode(randomX, randomY);      

                % Calculate new point in the tree
                newPoint = obj.generateNewNode(minNode, angle);

                % Check if the new point is within valid space and not in blocks
                if obj.isValidNode(newPoint(1), newPoint(2)) && ...
                   obj.checkSegmentIntersection(obj.treeNew(:, minNode), newPoint)

                    % Update the distance to the target and store the node
                    obj.targetDistance = norm(obj.targetPoint - newPoint');
                    obj.nodeNumber = obj.nodeNumber + 1;
                    obj.treeOld(:, obj.nodeNumber) = obj.treeNew(:, minNode);
                    obj.treeNew(:, obj.nodeNumber) = newPoint;
                end      
            end  

            obj.drawPath();
            elapsedTime = toc; % time ends

            % Store the final iteration count, nodes count, nodes in path, 
            % total length of path and time
            data = [obj.iteration, size(obj.treeNew, 2), obj.node, (obj.rrtPath) * obj.distance, elapsedTime];
     
            % Use the instance of Main to call recordData
            obj.mainInstance.recordData(data, 0);
        end                 
    end    
end
