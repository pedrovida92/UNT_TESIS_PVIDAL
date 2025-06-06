clc, clear
global qc;
global qa;
global tc;
global P_home;
global O_home;
home = dkine([0,0,0,0,0,0]);
P_home = home(1:3,4).';
O_home = [0,-90,180];
qa = [0,0,0,0,0,0];
qc = [0,0,0,0,0,0];
tc = 0;

motor_velocidad_nominal = 24000;

motor_aceleracion_max1 = rad2deg(3988.57663339054);
motor_aceleracion_max2 = rad2deg(21154.3016615268);
motor_aceleracion_max3 = rad2deg(15705.1189464815);
motor_aceleracion_max4 = rad2deg(11007.3431208998);
motor_aceleracion_max5 = rad2deg(14705.5967540500);
motor_aceleracion_max6 = rad2deg(6568.61702460416);

motor_jerk_max1 = rad2deg(37979.3276403546);
motor_jerk_max2 = rad2deg(1068338.24782041);
motor_jerk_max3 = rad2deg(588835.317751870);
motor_jerk_max4 = rad2deg(104812.199755351);
motor_jerk_max5 = rad2deg(742664.623051483);
motor_jerk_max6 = rad2deg(246278.535428709);

ratio1 = 94.5;   % 56/48*81
ratio2 = 262.6;   % 52/20*101
ratio3 = 153.9;   % 76/40*81
ratio4 = 97.2;   % 48/40*81
ratio5 = 63.75;  % 50/40*51
ratio6 = 51.0;

vel_max1 = motor_velocidad_nominal/ratio1;
vel_max2 = motor_velocidad_nominal/ratio2;
vel_max3 = motor_velocidad_nominal/ratio3;
vel_max4 = motor_velocidad_nominal/ratio4;
vel_max5 = motor_velocidad_nominal/ratio5;
vel_max6 = motor_velocidad_nominal/ratio6;

acel_max1 = motor_aceleracion_max1/ratio1;
acel_max2 = motor_aceleracion_max2/ratio2;
acel_max3 = motor_aceleracion_max3/ratio3;
acel_max4 = motor_aceleracion_max4/ratio4;
acel_max5 = motor_aceleracion_max5/ratio5;
acel_max6 = motor_aceleracion_max6/ratio6;

jerk_max1 = motor_jerk_max1/ratio1;
jerk_max2 = motor_jerk_max2/ratio2;
jerk_max3 = motor_jerk_max3/ratio3;
jerk_max4 = motor_jerk_max4/ratio4;
jerk_max5 = motor_jerk_max5/ratio5;
jerk_max6 = motor_jerk_max6/ratio6;

% global q1;
% global q2;
% global q3;
% global q4;
% global q5;
% global q6;
% 
% global dq1;
% global dq2;
% global dq3;
% global dq4;
% global dq5;
% global dq6;
% 
% global ddq1;
% global ddq2;
% global ddq3;
% global ddq4;
% global ddq5;
% global ddq6;
