clc, clear all, close all;

M = readmatrix("AP.csv");

L = {};

POINTS = {  [400,    0, 350, 0, -90, 180],
            [250,  150, 200, 0, -90, 180],
            [250, -150, 200, 0, -90, 180],
            [550, -150, 500, 0, -90, 180],
            [550,  150, 500, 0, -90, 180]};

% Static values
NUM_JOINTS = 6;
NUM_POINTS = 5;
NUM_CYCLES = length(M)/NUM_POINTS;
matrix_traj_i = zeros(NUM_CYCLES,NUM_JOINTS);

for i=1:NUM_POINTS
    for j=1:NUM_CYCLES
        matrix_traj_i(j,:) = M((j-1)*NUM_POINTS+i,:);
    end
    L{i} = matrix_traj_i;
end

for i=1:NUM_POINTS
    X = mean(L{i}(:,1));
    Y = mean(L{i}(:,2));
    Z = mean(L{i}(:,3));
    Pm = [X, Y, Z];
    AP{i} = Pm - POINTS{i}(1:3);
    AP_result(i) = norm(AP{i});

    for j=1:NUM_CYCLES
        P = L{i}(j,1:3);
        l(j) = norm([P-Pm]);
    end
    lm = mean(l);
    sum = 0;
    for j=1:NUM_CYCLES
        sum = sum + (l(j) - lm)^2;
    end
    S = sqrt(sum/(NUM_CYCLES-1));
    RP{i} = lm + 3*S;
end

for i=1:NUM_POINTS
    disp(L{i});
end

AP_result
RP




