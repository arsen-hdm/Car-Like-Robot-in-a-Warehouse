%sorts the neighbor into the appropriate spot in the OPEN matrix based on esttotcost
function result = Astar_newneighbor(OPEN,neighbor,esttotcost)
    result = zeros(size(OPEN));
    i = 1;
if OPEN(1,1) ~= 0 %if there is at least one node already in open
    while OPEN(i,2) < esttotcost && OPEN(i,1) ~= 0 %while esttotcost is greater than totcost of other nodes and there are still nodes to compare to
        result(i,1) = OPEN(i,1); %fill rows of result with rows of OPEN
        result(i,2) = OPEN(i,2);
        i = i + 1;
    end
    
    % adds the new node in the correct position
    if i <= size(OPEN,1)
        result(i,1) = neighbor;
        result(i,2) = esttotcost;
        i = i + 1;
    end
        
    % copies the other nodes in OPEN
    while i <= size(OPEN,1) && OPEN(i-1,1) ~= 0
        result(i,:) = OPEN(i-1,:);
        i = i + 1;
    end
 
else
    result(1,1) = neighbor; %if OPEN was empty put the new neighbor in the first row 
    result(1,2) = esttotcost;
    
end
end