%takes a matrix of nodes and a matrix of edges and returns the lowest cost path between the first and last node
function [path] = Astar(map,edges,nodes) 
solutionfound = 0;

% OPEN = zeros(10000,2); %initialize the OPEN and CLOSED matrices
% OPEN(1,1) = 1;
% CLOSED = zeros(10000,1);
% closedcount = 1;

nodessize = size(nodes);
numnodes = nodessize(1);
pastcost = zeros(numnodes,1); %initialize the pastcost and parent matrices
    for i = 2:numnodes
        pastcost(i) = 100;
    end
parent = zeros(numnodes,1);

OPEN = zeros(numnodes,2); %initialize the OPEN and CLOSED matrices
OPEN(1,1) = 1;
CLOSED = zeros(numnodes,1);
closedcount = 1;

goal_x = nodes(numnodes,2);
goal_y = nodes(numnodes,3);

while OPEN(1,1) ~= 0 %while OPEN has nodes in it explore the first one
    current = OPEN(1,1); %set the first node in OPEN to current and remove it from OPEN
    OPEN(1,:) = 0;
    OPEN = Astar_fixOPEN(OPEN); %moves all nonzero entries up one row
    CLOSED(closedcount) = current; %add the current node being explored to CLOSED
    closedcount = closedcount + 1;
    
    if current == numnodes %if node being explored is the goal node (because with the RRT it must be that the last node is also the goal one)
        path = Astar_pathfinder(parent,numnodes); %calculates final path based on parent matrix
        solutionfound = 1; 
    end
    
    edgessize = size(edges);
    numedges = edgessize(1);

    for i = 1:numedges %search each row in edges to see if it involves current node
        [neighbor,cost] = Astar_neighborfinder(current,edges(i,:)); %returns neighbor and cost if current is one of the nodes on the edge, else returns 0
        if neighbor ~= 0 && Astar_isitclosed(neighbor,CLOSED) == 0 %figure out if neighbor is in CLOSED
            tentative = pastcost(current) + cost;
            if tentative < pastcost(neighbor) %change pastcost and parent of node being explored if tentative is an improvement
                pastcost(neighbor) = tentative;
                parent(neighbor) = current;

                % computes the heuristic euclidian distance from the
                % neighbor to the goal
                h = sqrt((goal_x - nodes(neighbor,2))^2 + (goal_y - nodes(neighbor,3))^2);

                OPEN = Astar_newneighbor(OPEN,neighbor,pastcost(neighbor)+h); %resort OPEN matrix 
            end
        end
    end   
end  
    if solutionfound == 0 %if a solution wasn't found return 1
        path = 1;
    end

% Saves the files in that directory
writematrix(path, fullfile('Results', 'Astar_path.csv')); %write the path matrix into a csv

plotCSV(map,'Astar');

end