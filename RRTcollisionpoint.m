function result = RRTcollisionpoint(x, y, map)
    
    % m per cell
    resolution = 1;

    % origin in the lower left
    origin = [0, 0];

    % conversion of coordinates from world (x,y) -> indexes (row,column)

    col = floor((x - origin(1)) / resolution) + 1;
    row = size(map, 1) - floor((y - origin(2)) / resolution);
    
    % controlling if it's out of the map...

    if row < 1 || row > size(map,1) || col < 1 || col > size(map,2)
        result = 1; % if it is so -> considered as collision
        return
    end
    
    % controlling the cell...

    if map(row, col) == true
        result = 1; % in an obstacle
    else
        result = 0; % obstacle free
    end
end
