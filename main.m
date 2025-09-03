clear all
close all
clc

%% Map selection
maps = ["Normal", "Inflated"];

fprintf('Select a map configuration:\n');
for i = 1:length(maps)
    fprintf('  %d - %s\n', i, maps(i));
end

valid_choice = false;
while ~valid_choice
    choice = input('Type the number corresponding to the desired map: ');

    if isnumeric(choice) && isscalar(choice) && choice >= 1 && choice <= length(maps)
        valid_choice = true;
        choosen_map = maps(choice);
        fprintf('The choosen configuration is: %s\n', choosen_map);
    else
        fprintf('Choice is not admissible, make a choice between 1 and %d.\n', length(maps));
    end
end

path = 'Environment/';
if choosen_map == "Normal"
    load([path 'warehouseMap.mat']);
elseif choosen_map == "Inflated"
    load([path 'WarehouseMapInflated.mat']);
end

%% Model selection
models = ["Long", "Short"];

fprintf('Select a model type:\n');
for i = 1:length(models)
    fprintf('  %d - %s\n', i, models(i));
end

valid_choice = false;
while ~valid_choice
    choice = input('Type the number corresponding to the desired model: ');

    if isnumeric(choice) && isscalar(choice) && choice >= 1 && choice <= length(models)
        valid_choice = true;
        choosen_model = models(choice);
        fprintf('The choosen model is: %s\n', choosen_model);
    else
        fprintf('Choice is not admissible, make a choice between 1 and %d.\n', length(models));
    end
end

path = 'RobotModel/';
if choosen_model == "Long"
    load([path 'MiR250_long_l.mat']);
elseif choosen_model == "Short"
    load([path 'MiR250_short_l.mat']);
end

%% Planner selection
planners = ["RTT+A*", "RS"];

fprintf('Select a planner:\n');
for i = 1:length(planners)
    fprintf('  %d - %s\n', i, planners(i));
end

valid_choice = false;
while ~valid_choice
    choice = input('Type the number corresponding to the desired planner: ');

    if isnumeric(choice) && isscalar(choice) && choice >= 1 && choice <= length(planners)
        valid_choice = true;
        choosen_planner = planners(choice);
        fprintf('The choosen model is: %s\n', choosen_planner);
    else
        fprintf('Choice is not admissible, make a choice between 1 and %d.\n', length(planners));
    end
end

%% Feedback options
feedback_options = ["Runge-Kutta 2nd order", "Runge-Kutta 4th order"];

fprintf('Select a Feedback:\n');
for i = 1:length(feedback_options)
    fprintf('  %d - %s\n', i, feedback_options(i));
end

valid_choice = false;
while ~valid_choice
    choice = input('Type the number corresponding to the desired feedback strategy: ');

    if isnumeric(choice) && isscalar(choice) && choice >= 1 && choice <= length(feedback_options)
        valid_choice = true;
        feedback_choosen_option = feedback_options(choice);
        fprintf('You have choosen the: %s\n', feedback_choosen_option);
    else
        fprintf('Choice is not admissible, make a choice between 1 e %d.\n', length(feedback_options));
    end
end

%% Choosing among planners

if choosen_planner == "RTT+A*"
    %% RRT + A*
    startpos = [7,3];
    endpos = [43,38];
    startorient = [1.57,0];

    path_RRT = [];
    edges_RRT = [];
    nodes_RRT = [];

    if choosen_map == "Normal"
        [path_RRT,edges_RRT,nodes_RRT] = RRTplanner_new(logicalMap,startpos,endpos);
    elseif choosen_map == "Inflated"
        [path_RRT,edges_RRT,nodes_RRT] = RRTplanner_new(inflatedMap,startpos,endpos);
    end

    optimalpath_Astar = [];

    if choosen_map == "Normal"
        optimalpath_Astar = Astar(logicalMap,edges_RRT,nodes_RRT);
    elseif choosen_map == "Inflated"
        optimalpath_Astar = Astar(inflatedMap,edges_RRT,nodes_RRT);
    end

    %% Convert path IDs to coordinates
    ids = optimalpath_Astar;
    xp = zeros(length(ids),1);
    yp = zeros(length(ids),1);

    for k = 1:length(ids)
        idx = find(nodes_RRT(:,1) == ids(k));
        xp(k) = nodes_RRT(idx,2);  % x
        yp(k) = nodes_RRT(idx,3);  % y
    end

    xp=xp';
    yp=yp';

%% Reed-Sheeps Curves
elseif choosen_planner == "RS"
    startpos = [7,3];
    startorient = [1.57/2,0];
    %startpose = [startpos, startorient(1)];
    %secondpoint = [25, 16, 0];
    %thirdpoint = [41, 16, 0];
    %fourthpoint = [41, 8, pi*3/2];
    endpos = [34, 8, pi];
    %points = [startpose;secondpoint;thirdpoint;fourthpoint;endpos];
    %path = reeds_shepp_curve(points,phi_lim,l);
    % 2
    % xp = path(:,1)';
    % yp = path(:,2)';
    % 
    % save('desired_path_RS.mat', 'xp', 'yp');

    % startpos, startorient and endpos are always needed, but due to
    % the longer time needed for computing in real time the RS path
    % I've computed it offline and saved it so here it just loads it:

    load('desired_path_RS.mat');

    k = 10; % take a point for every 10 because it works better, in this case we're going from 500 values to almost 50
    xp = xp(1:k:end);
    yp = yp(1:k:end);


end

%% Controller parameters
b = 0.2;
k1 = 5;
k2 = 6;
delay = 3;

switch feedback_choosen_option
    case "Runge-Kutta 2nd order"
        open("Trajectory_tracking_2nd_order.slx");
        out = sim("Trajectory_tracking_2nd_order.slx");

    case "Runge-Kutta 4th order"
        open("Trajectory_tracking_4th_order.slx");
        out = sim("Trajectory_tracking_4th_order.slx");
end

%% Example plot section (adattabile)
personal_plot6(out.error_x.Time,out.error_x.Data(1,:),'Time (s)','Pos error x-axis (m)','x_error.pdf');
personal_plot6(out.error_y.Time,out.error_y.Data(1,:),'Time (s)','Pos error y-axis (m)','y_error.pdf');
personal_plot6(out.error_norm.Time,out.error_norm.Data,'Time (s)','Pos errorNorm (m)','error_norm.pdf');
personal_plot6(out.v.Time,out.v.Data,'Time (s)','v (m/s)','v.pdf');
personal_plot6(out.w.Time,out.w.Data,'Time (s)','w (rad/s)','w.pdf');
personal_plot6(out.e1.Time,out.e1.Data,'Time (s)','Error y1 (m)','y1_error.pdf'); 
personal_plot6(out.e2.Time,out.e2.Data,'Time (s)','Error y2 (m)','y2_error.pdf'); 

%% Final plot: map + planned path + robot's trajectory
figure;
hold on;

% --- Map ---
if choosen_map == "Normal"
    map = logicalMap;
elseif choosen_map == "Inflated"
    map = inflatedMap;
end

% Draws the map, obstacles in black and free space in white
imagesc(flipud(~map));
colormap(gray);
axis equal;
axis xy;            % origin at the lower left

% --- Starting point and destination point ---
plot(startpos(1), startpos(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g'); % start = green
plot(endpos(1), endpos(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');     % goal = red

% --- planned path ---
plot(xp, yp, 'b-', 'LineWidth',2);   % blue color for the planned path
plot(xp, yp, 'bo', 'MarkerSize',4, 'MarkerFaceColor','b'); % nodes on the path

% --- robot's trajectory due to the controller ---
plot(out.x.Data, out.y.Data, 'm-', 'LineWidth',2); % in purple

legend('Start','Goal','Planned path','Nodes','Real trajectory');
title('Planned path vs Real path');
xlabel('X (m)');
ylabel('Y (m)');