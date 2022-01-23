close all
clear
clc

%Model parameters
global m Ixx Iyy Izz Ir g L k b Ir_2
m = 0.468/4;      %Mass of model [kg]
Ixx = 4.856*10^(-3);    %X-axis moment of inertia [kg*m^2]
Iyy = 4.856*10^(-3);    %Y-axis moment of inertia [kg*m^2]
Izz = 8.801*10^(-3);    %Z-axis moment of inertia [kg*m^2]
Ir = 3.357*10^(-5);     %Moment of inertia of single motor [kg*m^2]
Ir_2 = 0; 

L = 0.225;   %Length of arm [m]
g = 9.81;    %Gravitational field strength [m/s^2]

k = 2.98*10^(-6);  %Lift constant 
b = 1.14*10^(-7);  %Drag constant 

%Aerodynamical resistance
global Ax Ay Az  %Drag force coeffitients 
Ax = 0.25;       %In x direction
Ay = 0.25;       %In y direction
Az = 0.25;       %In z direction

%Initial conditions 
x0 = 0;     %Initial x value [m]
y0 = 0;     %Initial y value [m]
z0 = 0;     %Initial z value [m]
Vx0 = 0;    %Initial Vx value [m/s]
Vy0 = 0;    %Initial Vy value [m/s]
vZ0 = 0;    %Initial Vz value [m/s]

%Controller parameters
global kpsi_p kphi_p ktheta_p kpsi_d kphi_d ktheta_d kz_d kz_p kxp kyp kzp kxd kyd kzd kxdd kydd kzdd

kxp = 1;
kyp = 1;
kzp = 1;
kxd = 3/4*2;
kyd = 3/4*2;
kzd = 3/4*2;
kxdd = 1;
kydd = 1;
kzdd = 1;

kz_p = 1.5;
kpsi_p = 3*1.5;  
kphi_p = 3*1.5;
ktheta_p = 3*1.5;
kz_d = 2.5;
kpsi_d = 0.75; 
kphi_d = 0.75;
ktheta_d = 0.75;

%Simulation parameters
global dT Tsim
dT = 0.01;  %Sample time
Tsim = 300;

%Errors in for D regulator
global e_theta_old e_phi_old e_psi_old
e_phi_old = -1;
e_theta_old = -1;
e_psi_old = -1;