function Q = iksolve(T,Qant)
    q = zeros(6,8);
    d = zeros(1,8);
    for i = 1:8
        conf  = dec2bin(8-i,3);
        %h = str2double(conf(1));
        h = 1;
        c = str2double(conf(2));
        m = str2double(conf(3));
        q(:,i) = ikine(T,h,c,m);
        d(i) = norm(q(:,i)-Qant);
    end
    [~,k] =  min(d);
    Q = q(:,k);
end