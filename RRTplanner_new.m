% takes an occupancy grid and finds a path from a starting position to a
% desired one using the RRT path algorithm
% also generates matrices of the nodes and edges created during the path
% search process
function [path,edges,nodes] = RRTplanner_new(map,startpos, endpos) 
pathfound = 0;

load('config.mat');

nodes = zeros(numnodes,3);
nodes(1,:) = [1, startpos(1), startpos(2)];
numnodes_actual = 1;

edges = []; % dynamic array
parent = zeros(numnodes,1);

k = 7; % max number of nearest nodes to connect to the current one (K-NN)

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
            
            % connecting to the nearest node (RRT)
            edges = [edges; index, numnodes_actual, RRTdistance(xsamp,ysamp,xnearest,ynearest)];
            parent(numnodes_actual) = index;

            % connecting to a certain number of nearest nodes (limited RRG)
            if numnodes_actual > 1
                dists = zeros(numnodes_actual-1,1);
                for i = 1:numnodes_actual-1
                    dists(i) = RRTdistance(xsamp,ysamp,nodes(i,2),nodes(i,3));
                end

                [sortedDists, idx] = sort(dists); % sorting by augmenting distance
                neighbors = idx(1:min(k,length(idx))); % take the first k nodes

                for i = neighbors'
                    if RRTcollisionline(xsamp,ysamp,nodes(i,2),nodes(i,3),map) == 0
                        dist = RRTdistance(xsamp,ysamp,nodes(i,2),nodes(i,3));
                        edges = [edges; i, numnodes_actual, dist];
                        edges = [edges; numnodes_actual, i, dist]; % optional: bi-directional arc
                    end
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

% plot of the results
plotCSV(map,'RRT');

end