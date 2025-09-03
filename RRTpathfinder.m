%finds the final path based on the parent matrix
function path = RRTpathfinder(parent,numnodes_actual)
    % numnodes = 2000;
    % Loading some variables of interest for the whole RRT algorithm
    load('config.mat', 'numnodes');

    bigmatrix = zeros(1,numnodes);
    bigmatrix(numnodes) = numnodes_actual;
    currentnode = numnodes_actual;
    i = numnodes-1;
    
    while currentnode ~= 1 %fill the oversized matrix with the path by backtracking from the last node
        bigmatrix(i) = parent(currentnode);
        currentnode = bigmatrix(i);
        i = i - 1;
    end
    
    path = zeros(1,numnodes-i); %create a matrix of correct path size based on how many elements added to bigmatrix
    i = i + 1;
    j = 1;
    
    while i < numnodes+1 %fill path matrix with path elements from bigmatrix
        path(j) = bigmatrix(i);
        i = i + 1;
        j = j + 1;
    end
end