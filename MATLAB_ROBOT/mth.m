function T = mth(P,O)
    T = [rpy2mat(O),P'; 0, 0, 0, 1];
end