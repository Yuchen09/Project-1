% @file Prm.m
% @brief Defines the PRM (Probabilistic Roadmap) algorithm for path planning.

%{
% @brief Prm - A subclass of BaseClass that implements the PRM algorithm.
% This class is used for generating a roadmap and finding a path using the A* algorithm.
%}

classdef Prm < BaseClass
    properties
        setMaps     %< Instance of the Maps class
        mapNumber   %< Number of the map being used
        PRMNode     %< Number of nodes to sample for the PRM algorithm
    end

    methods
        %{
        % @brief Constructor for the Prm class.
        % @param num The map number to be used for the PRM algorithm.
        %}
        function obj = Prm(num)
            obj.mapNumber = num;

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
            obj.setMaps = Maps(num); % Create Maps instance with the given number
            obj.setObstacles(obj.setMaps.space, obj.setMaps.blocks, obj.setMaps.walls); % Set obstacles
        end

        %{
        % @brief Receive the number of PRM nodes to sample.
        % @param PRMNode The number of nodes to sample.
        %}
        function receive(obj, PRMNode)
            obj.PRMNode = PRMNode;
        end

        %{
        % @brief Perform the PRM algorithm and find a path using A*.
        %}
        function performanceChecking(obj)

            % Sampling nodes
            nodes = [obj.startPoint; obj.targetPoint];
            obj.numSample = obj.PRMNode;

            while size(nodes, 1) < obj.numSample + 2
                new_node = [obj.space(1)+1  + (obj.space(2) - obj.space(1)-2) * rand(), ...
                            obj.space(3)+1 + (obj.space(4) - obj.space(3)-2) * rand()];
                if ~obj.check_collision(new_node)
                    nodes = [nodes; new_node];
                end
            end

            % Connect nodes to form a graph
            adj_matrix = zeros(size(nodes, 1)); % Adjacency matrix

            for i = 1:size(nodes, 1)
                for j = i+1:size(nodes, 1)
                    if norm(nodes(i, :) - nodes(j, :)) <= obj.radius
                        if obj.checkSegmentIntersection(nodes(i, :), nodes(j, :))
                            adj_matrix(i, j) = 1;
                            adj_matrix(j, i) = 1;
                        end
                    end
                end
            end

            % Find the path using A* algorithm
            [start_idx, target_idx] = deal(1, 2);
            [path, ~] = obj.astar(adj_matrix, nodes, start_idx, target_idx);

            for i = 2:length(path)
                plot([nodes(path(i-1), 1), nodes(path(i), 1)], ...
                     [nodes(path(i-1), 2), nodes(path(i), 2)], 'r-', 'LineWidth', 2 * obj.resolution);
            end
        end
    end
end
