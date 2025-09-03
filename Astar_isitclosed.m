function result = Astar_isitclosed(neighbor, CLOSED)
    result = any(CLOSED(:) == neighbor);   % returns 1 (true) or 0 (false)
end