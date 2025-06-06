function M = mdh(theta, d, a, alpha)
    ct = cos(theta);    
    st = sin(theta);
    ca = cos(alpha);    
    sa = sin(alpha);
    
    M = [ ct, -ca*st,  sa*st, a*ct;
          st,  ca*ct, -sa*ct, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
end
