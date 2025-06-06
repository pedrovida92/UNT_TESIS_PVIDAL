global q1;
global q2;
global q3;
global q4;
global q5;
global q6;

global dq1;
global dq2;
global dq3;
global dq4;
global dq5;
global dq6;

global ddq1;
global ddq2;
global ddq3;
global ddq4;
global ddq5;
global ddq6;

tf = tc;
t = M(:,1);
t0 = t(1);

q1 = [t, M(:,2)];
dq1 = [t, M(:,8)];
ddq1 = [t, M(:,14)];

q2 = [t, M(:,3)];
dq2 = [t, M(:,9)];
ddq2 = [t, M(:,15)];

q3 = [t, M(:,4)];
dq3 = [t, M(:,10)];
ddq3 = [t, M(:,16)];

q4 = [t, M(:,5)];
dq4 = [t, M(:,11)];
ddq4 = [t, M(:,17)];

q5 = [t, M(:,6)];
dq5 = [t, M(:,12)];
ddq5 = [t, M(:,18)];

q6 = [t, M(:,7)];
dq6 = [t, M(:,13)];
ddq6 = [t, M(:,19)];

n = length(t);

f0_gz = [0, 0, 9.80665];
f0_gy = [0, 9.80665, 0];
mu0 = [0,0,0];

tau_gz = zeros(n,6);
tau_gy = zeros(n,6);

mu_gz = zeros(n,6);
mu_gy = zeros(n,6);

fz = zeros(n,6);
fy = zeros(n,6);

for i=1:n
    [tau_gz(i,:),mu_gz(i,:),fz(i,:)] = dynamic_gz(M(i,2:7),M(i,8:13),M(i,14:19),f0_gz,mu0);
    [tau_gy(i,:),mu_gy(i,:),fy(i,:)] = dynamic_gy(M(i,2:7),M(i,8:13),M(i,14:19),f0_gy,mu0);
end
Tz = [t,tau_gz];
Ty = [t,tau_gy];

RS = sim('KR3_R540_TCP', "StartTime", num2str(t0), "StopTime", num2str(tf), "SaveOutput", 'on');
%T = [RS.tout,RS.simout];
