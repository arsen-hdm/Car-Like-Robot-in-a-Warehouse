% function result = RRTnozeros(m) %removes extra zeros from unused rows of edges and nodes matrices
%     % numnodes = 2000;
%     % Loading some variables of interest for the whole RRT algorithm
% 
%     load('config.mat', 'numnodes');
%     bigmatrix = zeros(numnodes,3);
%     i = 1;
%     while m(i,1) ~= 0
%         bigmatrix(i,1) = m(i,1);
%         bigmatrix(i,2) = m(i,2);
%         bigmatrix(i,3) = m(i,3);
%         i = i + 1;
%     end
% 
%     i = i - 1;
%     result = zeros(i,3);
% 
%     for j = 1:i
%         result(j,1) = bigmatrix(j,1);
%         result(j,2) = bigmatrix(j,2);
%         result(j,3) = bigmatrix(j,3);
%     end
% end

function result = RRTnozeros(m)
    % searches among the rows until finds one with a zero element in the first column 
    idx = find(m(:,1) == 0, 1, 'first'); 
    
    if isempty(idx)
        result = m; % if there's not a zero keep the entire matrix
    else
        result = m(1:idx-1,:); % keeps only the non null rows
    end
end
