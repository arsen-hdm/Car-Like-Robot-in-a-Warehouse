%takes a occupancy grid and finds a path from a starting position to a
%desired one using the RRT path algorithm
%also generates matrices of the nodes and edges created during the path
%search process
function [path,edges,nodes] = RRTplanner(map,startpos, endpos) 
pathfound = 0;

load('config.mat');

nodes = zeros(numnodes,3);
nodes(1,:) = [1, startpos(1), startpos(2)];
numnodes_actual = 1;

edges = []; % dynamic array
parent = zeros(numnodes,1);

r = 5; % radius in m to connect nearest nodes

while numnodes_actual < numnodes && pathfound == 0
    xsamp = rand*50;
    ysamp = rand*50;
    
    if rand < goalbias
        xsamp = endpos(1);
        ysamp = endpos(2);
    end
    
    if RRTcollisionpoint(xsamp,ysamp,map) == 0
        [index,xnearest,ynearest] = RRTnearest(xsamp,ysamp,nodes,numnodes_actual);
        
        if RRTcollisionline(xsamp,ysamp,xnearest,ynearest,map) == 0
            numnodes_actual = numnodes_actual + 1;
            nodes(numnodes_actual,:) = [numnodes_actual, xsamp, ysamp];
            
            % connection to the nearest node (RRT)
            edges = [edges; index, numnodes_actual, RRTdistance(xsamp,ysamp,xnearest,ynearest)];
            parent(numnodes_actual) = index;

            % connections of other nodes that are in the circunference of radius r (RRG)
            for i = 1:numnodes_actual-1
                dist = RRTdistance(xsamp,ysamp,nodes(i,2),nodes(i,3));
                if dist <= r && RRTcollisionline(xsamp,ysamp,nodes(i,2),nodes(i,3),map) == 0
                    edges = [edges; i, numnodes_actual, dist];
                    edges = [edges; numnodes_actual, i, dist]; % optional, bi-directional arc
                end
            end
            
            % goal control
            if xsamp == endpos(1) && ysamp == endpos(2)
                pathfound = 1;
                path = RRTpathfinder(parent,numnodes_actual);
            end
        end
    end
end

edges = RRTnozeros(edges);
nodes = RRTnozeros(nodes);


% Name of the destination folder
results_folder = 'Results';

% Create it if it doesn't exist
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

% Saves the files in that directory
writematrix(edges, fullfile(results_folder, 'edges.csv'));
writematrix(nodes, fullfile(results_folder, 'nodes.csv'));
writematrix(path,  fullfile(results_folder, 'RRTpath.csv'));

% % Number of nodes and edges
% num_nodes = size(nodes, 1);
% num_edges = size(edges, 1);

% % Calculation of the lenght of the path in m
% path_length = 0;
% for i = 1:length(path)-1
%     n1 = path(i);
%     n2 = path(i+1);
%     p1 = nodes(n1, 2:3);  % coordinates of node 1
%     p2 = nodes(n2, 2:3);  % coordinates of node 2
%     path_length = path_length + norm(p2 - p1);
% end
% 
% % Number of nodes and their ID
% num_path_nodes = length(path);
% path_node_ids = path(:)';  % IDs
% 
% % Print of the results
% fprintf('Number of total nodes: %d\n', num_nodes);
% fprintf('Number of total edges: %d\n', num_edges);
% fprintf('Number of the nodes through the path: %d\n', num_path_nodes);
% fprintf('IDs of the nodes belonging to the path: %s\n', num2str(path_node_ids));
% fprintf('Total lenght of the path: %.2f m\n', path_length);

plotCSV(map,'RRT');

end