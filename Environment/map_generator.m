close all; clc;

% Map dimensions
rows = 50;
columns = 50;

% Initialization of a new map
logicalMap = false(rows, columns);

% Generation of some obstacles and walls

logicalMap(1, 1:columns) = true;  % upper wall
logicalMap(1:rows, 1) = true;  % left wall
logicalMap(rows, 1:columns) = true;  % lower wall
logicalMap(1:rows, columns) = true;  % rigth wall

logicalMap((rows/2), (columns/2)+5:columns) = true;  % wall between storage room 1 and 2

logicalMap(1:((rows/2)-13), (columns/2)+5) = true;  % wall of room 1
logicalMap(((rows/2)-7):((rows/2)+7), (columns/2)+5) = true;  % wall of room 1 and 2

logicalMap(((rows/2)+13:rows), (columns/2)+5) = true;  % wall of room 2

logicalMap(6:7, 1:12) = true;    % conveyor belt
logicalMap(7:33, 11:12) = true;  
logicalMap(33:36, 9:14) = true;  % sorting station

logicalMap(47:49, 2:5) = true;  % charging station of the robot

logicalMap(49, 23:29) = true;  % some random boxes
logicalMap(43:49, 29) = true;  
logicalMap(48, 24:28) = true;  
logicalMap(44:48, 28) = true;  
logicalMap(47, 25:27) = true;  
logicalMap(45:47, 27) = true;  
logicalMap(46, 26) = true;

logicalMap(6:12, 20:22) = true; % some shelfs
logicalMap(22:28, 20:22) = true;

logicalMap(3:5, 35:48) = true;  % shelf in room 1
logicalMap(5:23, 46:48) = true;
logicalMap(21:23, 35:48) = true;

logicalMap(27:29, 35:48) = true; % shelf in room 2
logicalMap(30:48, 46:48) = true;
logicalMap(46:48, 35:48) = true;


% Save in .mat
%save('warehouseMap.mat', 'logicalMap');

%load warehouseMaps.mat logicalMap
%map = binaryOccupancyMap(logicalMap);
%show(map)

% Inflaction of the obstacles

safety_margin_m = 0.1;   % margin in m
map_resolution = 1;      % m for cell
margin_cells = ceil(safety_margin_m / map_resolution);

% circular mask so to inflate near the obstacles
se = strel('disk', margin_cells);

% "inflated" map
inflatedMap = imdilate(logicalMap, se);

% Visualization

% original map
map_orig = binaryOccupancyMap(logicalMap);
figure;
subplot(1,2,1);
show(map_orig);
title('original map');
% chargingStation = [5,5];
% loadingStation = [11,11];
% conveyorbelt = [1,30];
% shelf1 = [36,41];
% shelf2 = [36,11];
% %Show the various locations on the map.
% 
% hold on;
% 
% text(chargingStation(1), chargingStation(2), 1, 'Charging Station');
% plotTransforms([chargingStation, 0], [1 0 0 0])
% 
% text(loadingStation(1), loadingStation(2), 1, 'Loading Station');
% plotTransforms([loadingStation, 0], [1 0 0 0])
% 
% text(conveyorbelt(1), conveyorbelt(2), 1, 'Conveyor Belt');
% plotTransforms([conveyorbelt, 0], [1 0 0 0])
% 
% text(shelf1(1), shelf1(2), 1, 'Shelf in room 1');
% plotTransforms([shelf1, 0], [1 0 0 0])
% 
% text(shelf2(1), shelf2(2), 1, 'Shelf in room 2');
% plotTransforms([shelf2, 0], [1 0 0 0])

% inflated map
map_infl = binaryOccupancyMap(inflatedMap);
subplot(1,2,2);
show(map_infl);
title(sprintf('inflated map with margin %.1f m', safety_margin_m));

% Save in .mat
save('WarehouseMapInflated.mat', 'inflatedMap');