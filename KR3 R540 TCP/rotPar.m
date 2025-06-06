function [ R ] = rotPar(theta,r)
% theta = theta*pi/180;
ct = cos(theta);
st = sin(theta);
rx  = r(1);
ry = r(2);
rz = r(3);
rxy = rx*ry;
ryz = ry*rz;
rxz = rx*rz;
uct = 1-ct;
R = [  rx^2*uct+ct   ,  rxy*uct-rz*st,  rxz*uct+ry*st;
        rxy*uct+rz*st, ry^2*uct+ct   ,  ryz*uct-rx*st;
        rxz*uct-ry*st,  ryz*uct+rx*st, rz^2*uct+ct   ];
end