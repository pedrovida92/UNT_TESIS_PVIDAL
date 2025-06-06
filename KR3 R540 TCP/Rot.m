function [theta,r] = Rot(Ri,Rf)
R = Ri.'*Rf;
ct = 0.5*(R(1,1)+R(2,2)+R(3,3)-1);
st = -sqrt(1-ct^2);
theta = atan2(st,ct);
r = (1/(2*st))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
end