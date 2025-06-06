function out = evalz(in)
    [m, n] = size(in);
    for i = 1:m
        for j = 1:n
            if abs(in(i,j)) <= 1e-8
                in(i,j) = 0;
            end
        end
    end
    out = in;
end