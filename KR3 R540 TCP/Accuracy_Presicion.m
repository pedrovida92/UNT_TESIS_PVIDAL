AP = zeros(30,6);
k = 1;
for i=1:30
    AP(i,:) = AP_1(k,:);
    k = k+5;
end

AP_mean = mean(AP);

P = [400,0,350];
Presicion = sqrt((AP_mean(1)-P(1))^2+(AP_mean(2)-P(2))^2+(AP_mean(1)-P(1))^2);


