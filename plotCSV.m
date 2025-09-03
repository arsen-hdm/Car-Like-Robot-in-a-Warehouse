function plotCSV(map, pathType)
    % Results folder
    results_folder = 'Results';

    % Loading CSV files
    nodes = readmatrix(fullfile(results_folder, 'nodes.csv'));
    edges = readmatrix(fullfile(results_folder, 'edges.csv'));

    % Path selection
    if strcmpi(pathType, 'RRT')
        path = readmatrix(fullfile(results_folder, 'RRTpath.csv'));
        pathColor = 'k';
    elseif strcmpi(pathType, 'Astar')
        path = readmatrix(fullfile(results_folder, 'Astar_path.csv'));
        pathColor = 'k';
    else
        error('pathType must be "RRT" or "Astar"');
    end

    % Info
    disp(['Nodes size: ', num2str(size(nodes))]);
    disp(['Edges size: ', num2str(size(edges))]);
    disp([pathType, ' path length: ', num2str(length(path))]);

    % Visualization
    figure; clf; hold on;
    imagesc([0 50], [0 50], flipud(map));
    set(gca,'YDir','normal'); 
    axis equal tight;
    colormap(flipud(gray));
    xlabel('X [m]');
    ylabel('Y [m]');
    title(['Path with ', pathType, ' on the occupancy grid']);

    % Draw RRT tree
    for i = 1:size(edges,1)
        from_node = edges(i,1);
        to_node = edges(i,2);
        plot([nodes(from_node,2), nodes(to_node,2)], ...
             [nodes(from_node,3), nodes(to_node,3)], ...
             'b-', 'LineWidth', 0.5);
    end

    % Draw all nodes
    plot(nodes(:,2), nodes(:,3), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r');

    % Draw selected path
    for i = 1:length(path)-1
        n1 = path(i);
        n2 = path(i+1);
        plot([nodes(n1,2), nodes(n2,2)], ...
             [nodes(n1,3), nodes(n2,3)], ...
             '-', 'Color', pathColor, 'LineWidth', 2);
    end

    hold off;
    drawnow;
end