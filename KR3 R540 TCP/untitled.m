clc, clear

% CARACTERÍSTICAS MECANICAS DE LOS MOTORES BLDC

velocidad_nominal = 4000*pi/30; % [rad/s]

torque_continuo1 = 0.063; %[N.m]
torque_continuo2 = 0.125; %[N.m]
torque_continuo3 = 0.188; %[N.m]

torque_max1 = 0.18; %[N.m]
torque_max2 = 0.38; %[N.m]
torque_max3 = 0.60; %[N.m]

motor_inercia_rotor1 = 24e-7; % [kg.m2]
motor_inercia_rotor2 = 48e-7; % [kg.m2]
motor_inercia_rotor3 = 72e-7; % [kg.m2]

correa_ratio1 = 56/48;
correa_ratio2 = 52/20;
correa_ratio3 = 76/40;
correa_ratio4 = 48/40;
correa_ratio5 = 50/40;
correa_ratio6 = 1;

reductor_inercia_rotor1 = 54355.44e-9;
reductor_inercia_rotor2 = 11404.67e-9;
reductor_inercia_rotor3 = 11404.67e-9;
reductor_inercia_rotor4 = 9440.72e-9;
reductor_inercia_rotor5 = (1455.99+1487.89)*1e-9;
reductor_inercia_rotor6 = 7191.06e-9;

inercia_equivalente1 = motor_inercia_rotor3 + reductor_inercia_rotor1/(correa_ratio1^2);
inercia_equivalente2 = motor_inercia_rotor3 + reductor_inercia_rotor2/(correa_ratio2^2);
inercia_equivalente3 = motor_inercia_rotor2 + reductor_inercia_rotor3/(correa_ratio3^2);
inercia_equivalente4 = motor_inercia_rotor2 + reductor_inercia_rotor4/(correa_ratio4^2);
inercia_equivalente5 = motor_inercia_rotor1 + reductor_inercia_rotor5/(correa_ratio5^2);
inercia_equivalente6 = motor_inercia_rotor1 + reductor_inercia_rotor6/(correa_ratio6^2);

aceleracion_max1 = torque_continuo3/(inercia_equivalente1);
aceleracion_max2 = torque_continuo3/(inercia_equivalente2);
aceleracion_max3 = torque_continuo2/(inercia_equivalente3);
aceleracion_max4 = torque_continuo2/(inercia_equivalente4);
aceleracion_max5 = torque_continuo1/(inercia_equivalente5);
aceleracion_max6 = torque_continuo1/(inercia_equivalente6);

ratio_acel_vel1 = aceleracion_max1 / velocidad_nominal;
ratio_acel_vel2 = aceleracion_max2 / velocidad_nominal;
ratio_acel_vel3 = aceleracion_max3 / velocidad_nominal;
ratio_acel_vel4 = aceleracion_max4 / velocidad_nominal;
ratio_acel_vel5 = aceleracion_max5 / velocidad_nominal;
ratio_acel_vel6 = aceleracion_max6 / velocidad_nominal;

jerk1 = ratio_acel_vel1*aceleracion_max1;
jerk2 = ratio_acel_vel2*aceleracion_max2;
jerk3 = ratio_acel_vel3*aceleracion_max3;
jerk4 = ratio_acel_vel1*aceleracion_max4;
jerk5 = ratio_acel_vel2*aceleracion_max5;
jerk6 = ratio_acel_vel3*aceleracion_max6;
