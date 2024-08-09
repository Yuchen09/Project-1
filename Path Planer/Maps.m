% @file Maps.m
% @brief Define various map configurations with space and obstacles.

%{
% @brief Maps - A subclass of BaseClass that defines different map configurations.
% The class sets up space, walls, and blocks based on the input parameter "num".
%}

classdef Maps < BaseClass
    properties
        
    end
    
    methods 
        %{
        % @brief Constructor for the Maps class.
        % @param num The map number to define the configuration of space, walls, and blocks.
        %}
        function obj = Maps(num)
            % Define space and walls
            obj.space = [0, 30, 0, 30];
            obj.walls = [
                0, 29, 29, 0, 0, 0, 1, 1;
                29, 30, 30, 29, 0, 0, 29, 29;
                30, 30, 1, 1, 29, 30, 30, 29;
                1, 0, 0, 1, 30, 30, 1, 1
            ];

            % Define blocks based on input num
            switch num
                %{
                % @brief Configuration for Map 1.
                %}
                case 1
                    obj.blocks = [
                        obj.space(1) + 5, obj.space(1) + 10, obj.space(1) + 10, obj.space(1) + 5, obj.space(3) + 5, obj.space(3) + 5, obj.space(3) + 10, obj.space(3) + 10;
                        obj.space(1) + 15, obj.space(1) + 25, obj.space(1) + 25, obj.space(1) + 15, obj.space(3) + 10, obj.space(3) + 10, obj.space(3) + 12, obj.space(3) + 12;
                        obj.space(1) + 7, obj.space(1) + 10, obj.space(1) + 10, obj.space(1) + 7, obj.space(3) + 15, obj.space(3) + 15, obj.space(3) + 25, obj.space(3) + 25
                    ];

                %{
                % @brief Configuration for Map 2.
                %}
                case 2
                    obj.blocks = [
                        obj.space(1) + 5, obj.space(1) + 10, obj.space(1) + 8, obj.space(1) + 3, obj.space(3) + 6, obj.space(3) + 11, obj.space(3) + 15, obj.space(3) + 10;
                        obj.space(1) + 25, obj.space(1) + 27, obj.space(1) + 27, obj.space(1) + 25, obj.space(3) + 12, obj.space(3) + 12, obj.space(3) + 22, obj.space(3) + 22;
                        obj.space(1) + 13, obj.space(1) + 15, obj.space(1) + 10, obj.space(1) + 8, obj.space(3) + 18, obj.space(3) + 20, obj.space(3) + 25, obj.space(3) + 23
                    ];

                %{
                % @brief Configuration for Map 3.
                %}
                case 3
                    obj.blocks = [
                        obj.space(1) + 8, obj.space(1) + 18, obj.space(1) + 18, obj.space(1) + 8, obj.space(3) + 23, obj.space(3) + 23, obj.space(3) + 24, obj.space(3) + 24;
                        obj.space(1) + 8, obj.space(1) + 13, obj.space(1) + 14, obj.space(1) + 9, obj.space(3) + 23, obj.space(3) + 18, obj.space(3) + 19, obj.space(3) + 24;
                        obj.space(1) + 7, obj.space(1) + 8, obj.space(1) + 8, obj.space(1) + 7, obj.space(3) + 10, obj.space(3) + 10, obj.space(3) + 15, obj.space(3) + 15;
                        obj.space(1) + 3, obj.space(1) + 7, obj.space(1) + 7, obj.space(1) + 3, obj.space(3) + 14, obj.space(3) + 14, obj.space(3) + 15, obj.space(3) + 15;
                        obj.space(1) + 14, obj.space(1) + 15, obj.space(1) + 25, obj.space(1) + 24, obj.space(3) + 8, obj.space(3) + 7, obj.space(3) + 17, obj.space(3) + 18; 
                    ];

                %{
                % @brief Configuration for Map 4.
                %}
                case 4
                    obj.blocks = [
                        obj.space(1) + 8, obj.space(1) + 18, obj.space(1) + 18, obj.space(1) + 8, obj.space(3) + 23, obj.space(3) + 23, obj.space(3) + 24, obj.space(3) + 24;
                        obj.space(1) + 7, obj.space(1) + 8, obj.space(1) + 8, obj.space(1) + 7, obj.space(3) + 7, obj.space(3) + 7, obj.space(3) + 17, obj.space(3) + 17;
                        obj.space(1) + 3, obj.space(1) + 13, obj.space(1) + 13, obj.space(1) + 3, obj.space(3) + 12, obj.space(3) + 12, obj.space(3) + 13, obj.space(3) + 13;
                        obj.space(1) + 18, obj.space(1) + 24, obj.space(1) + 24, obj.space(1) + 24, obj.space(3) + 4, obj.space(3) + 4, obj.space(3) + 18, obj.space(3) + 18;
                        obj.space(1) + 16, obj.space(1) + 21, obj.space(1) + 22, obj.space(1) + 17, obj.space(3) + 12, obj.space(3) + 7, obj.space(3) + 8, obj.space(3) + 13; 
                    ];

                %{
                % @brief Configuration for Map 5.
                %}
                case 5
                    obj.blocks = [
                        obj.space(1) + 6, obj.space(1) + 12, obj.space(1) + 11, obj.space(1) + 10, obj.space(3) + 19, obj.space(3) + 16, obj.space(3) + 25, obj.space(3) + 27;
                        obj.space(1) + 5, obj.space(1) + 5.7, obj.space(1) + 10.5, obj.space(1) + 11.6, obj.space(3) + 15, obj.space(3) + 13.1, obj.space(3) + 6.7, obj.space(3) + 7.6;
                        obj.space(1) + 17, obj.space(1) + 18, obj.space(1) + 23, obj.space(1) + 22, obj.space(3) + 17, obj.space(3) + 15, obj.space(3) + 24, obj.space(3) + 23.5;
                        obj.space(1) + 18, obj.space(1) + 17, obj.space(1) + 22, obj.space(1) + 24, obj.space(3) + 17, obj.space(3) + 15, obj.space(3) + 4, obj.space(3) + 3.5;
                    ];

                %{
                % @brief Handle invalid map numbers.
                %}
                otherwise
                    error('Invalid input number.');
            end
        end
    end
end
