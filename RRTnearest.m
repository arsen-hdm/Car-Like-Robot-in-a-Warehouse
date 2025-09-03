function [index,xnearest,ynearest] = RRTnearest(x,y,nodes,numnodes) %finds the nearest node to the sampled point
    xnearest = nodes(1,2);
    ynearest = nodes(1,3);
    index = 1;
    dist = RRTdistance(x,y,xnearest,ynearest);
    
    for i = 2:numnodes %iterates through the nodes matrix to find the closest node
        if RRTdistance(x,y,nodes(i,2),nodes(i,3)) < dist
            xnearest = nodes(i,2);
            ynearest = nodes(i,3);
            index = i;
            dist = RRTdistance(x,y,nodes(i,2),nodes(i,3));
        end
    end
end