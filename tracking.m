classdef tracking < matlab.System
    % Public, tunable properties
    properties
        filters = {};
        % ukf;
    end

    % Pre-computed constants
    properties(Access = private)
        cfg
    end

    methods
        % Constructor
        function obj = tracking(config)
            % Support name-value pair arguments when constructing object
            obj.cfg = config;
        end
    end

    methods(Access = public)
        %% Common functions
        function [newFilters] = track(obj, data)
            m = size(data, 1);
            f = length(obj.filters);
            
            % create a matrix for calculating distance between measurements
            % and predictions
            
            % additional rows for intializing filters (weights by 1/ large 
            % number
            w_ij = 10*zeros(m, f+m);
            
            % get only the polar cordinartes
            measurements = squeeze(data(:, :, 2)); % [M, NX]
            % get likelihood of measurements within track pdfs
            for i=1:m
                for j=1:f
                    w_ij(i, j) = obj.filters(j, 1).likelihood(measurements(m, 1));
                end 
            end
            
            
             % Weights for initializing new filters
            for j=f+1 : m+f
                w_ij(j-f, j) = 1 / (10000 * 10000);  % small number nearest to 0
            end
            
            % Solve the maximum-sum-weights problem (i.e., assignment problem)
            % Use global nearest neighbour by minimizing the distances
            % Overall Measurement-filter-associations
            [assignment, cost] = munkres(w_ij)
            
            cnt = 0;
            for x = 1:length(assignment)
                cnt = cnt+1;
                measurment = squeeze(data(x, :, :));
                if assignment(x) <= f
                    obj.filters(assignment(x), 1).update(measurment);
                    newFilters(cnt, 1) = obj.filters(assignment(x), 1);
                else
                    newFilters(cnt, 1) = Ukf(obj.cfg, measurment)
                end
            end
            
            % .filters = newFilters;
            
        end
    end
end
