clc, clear

f0 = [0, 0, 9.80665];
mu0 = [0,0,0];
q = deg2rad([0,-90, 85.6, 0, 4.4, 0]);
qp = [0,0,0,0,0,0];
qpp = [0,0,0,0,0,0];

tau = dynamic_gz(q,qp,qpp,f0,mu0)