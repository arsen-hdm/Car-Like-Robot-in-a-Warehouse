%move all elements in OPEN up by one row
% function new = fixOPEN(OPEN)
% 
% i = 1;
% while OPEN(i+1,1) ~= 0
%     OPEN(i,1) = OPEN(i+1,1);
%     OPEN(i,2) = OPEN(i+1,2);
%     i = i + 1;
% end
% OPEN(i,1) = 0;
% OPEN(i,2) = 0;
% new = OPEN;
% end

function new = Astar_fixOPEN(OPEN)
% keep only non null rows and pad zeros at the bottom
rows = OPEN(:,1) ~= 0;   % finds non null rows
new = [OPEN(rows,:); zeros(sum(~rows),2)]; % brings them up and pad the rest
end
