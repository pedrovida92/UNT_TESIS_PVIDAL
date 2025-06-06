function [ R ] = rpy2mat (rpy)
rpy = rpy*pi/180;
phi = rpy(1);
theta = rpy(2);
psi = rpy(3);
R = rotz(phi)*roty(theta)*rotx(psi);
R = evalz(R);
end