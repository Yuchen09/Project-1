% @file Main.m
% @brief Class for testing the performance of RRT, RRT*, and PRM algorithms

%{
% @brief Main - A class for testing and comparing the performance of RRT, RRT*, and PRM algorithms.
% This class runs multiple tests for each algorithm and calculates the average performance metrics.
% Inherits from BaseClass.
%}

classdef Main < BaseClass
    properties
        rrtData      %< Stores performance data for RRT
        rrtStarData  %< Stores performance data for RRT*
    end

    methods 
        %{
        % @brief Constructor for the Main class. 
        % This constructor initializes the performance testing for RRT, RRT*, and PRM algorithms.
        %}
        function obj = Main()
            maps = 5; % Number of maps 
            numRums = 1000; % Number of runs per map
       
            for n = 1:maps
                % Initialize performance data for each map
                obj.rrtData = zeros(1, 5);
                obj.rrtStarData = zeros(1, 5);

                for m = 1:numRums
                    % Perform RRT algorithm testing
                    rrtInstance = Rrt(n, obj);  
                    rrtInstance.performanceChecking();  
                    
                    % Perform RRT* algorithm testing
                    rrtStarInstance = RrtStar(n, obj);  
                    rrtStarInstance.performanceChecking();  

                    % PRM algorithm currently unavailable, still unfinished
                    % PrmInstance = Prm(n);  
                    % PrmInstance.performanceChecking(); 
                end
              
                % Calculate the average performance data for RRT and RRT*
                obj.rrtData = obj.rrtData / numRums;
                obj.rrtStarData = obj.rrtStarData / numRums;
           
                fprintf('Map:%d \n', n);
                % The following numbers represent the following:
                % Average (iteration; nodes in map; nodes in path; path length; time) 
                fprintf('Average (iteration; nodes in map; nodes in path; path length; time): \n');
                fprintf('RRT performance test: \n'); 
                disp(obj.rrtData); 
                fprintf('RRT* performance test:\n'); 
                disp(obj.rrtStarData);
            end
        end

        %{
        % @brief Record the performance data for RRT and RRT* algorithms.
        % @param rrt Performance data for RRT algorithm.
        % @param rrtStar Performance data for RRT* algorithm.
        %}
        function recordData(obj, rrt, rrtStar)
            % Store the RRT performance data  
            obj.rrtData = obj.rrtData + rrt();

            % Store the RRT* performance data
            obj.rrtStarData = obj.rrtStarData + rrtStar();
        end
    end
end
