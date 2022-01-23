function [ output ] = Controller( input )

%Controller is transforming informations: desired position to control forces
global Ixx Iyy Izz m g kpsi_d kphi_d ktheta_d kpsi_p ktheta_p kphi_p e_phi_old e_theta_old e_psi_old dT kxp kyp kzp kxd kyd kzd kxdd kydd kzdd

%Desired cartesian Position, Velocity, Acceleration and Yaw
x_d = input(1);
y_d = input(2);
z_d = input(3);

x_d_p = input(4);
y_d_p = input(5);
z_d_p = input(6);

x_d_b = input(7);
y_d_b = input(8);
z_d_b = input(9);

psi_d =  atan2(sin(input(10)),cos(input(10)));

%Actual position and angles
x = input(11); 
y = input(12);
z = input(13);
psi = atan2(sin(input(14)),cos(input(14)));
phi = atan2(sin(input(15)),cos(input(15)));
theta = atan2(sin(input(16)),cos(input(16)));

%Actual velocities (linear and angular)
x_p = input(17);
y_p = input(18);
z_p = input(19);
psi_p = input(20);
phi_p = input(21);
theta_p = input(22);
x_b = input(23);
y_b = input(24);
z_b = input(25);

%Compute errors
e_x = x_d - x; 
e_y = y_d - y;
e_z = z_d - z;
e_psi = psi_d - psi;
e_x_p = x_d_p - x_p;
e_y_p = y_d_p - y_p;
e_z_p = z_d_p - z_p;
e_x_b = x_d_b - x_b;
e_y_b = y_d_b - y_b;
e_z_b = z_d_b - z_b;

%Calculate d parameters for integrated PID controller

dx = kxp*e_x + kxd*e_x_p + kxdd*e_x_b;
dy = kyp*e_y + kyd*e_y_p + kydd*e_y_b;
dz = kzp*e_z + kzd*e_z_p + kzdd*e_z_b;

%Compute desired angular position
 phi_d = asin((dx*sin(psi)-dy*cos(psi))/sqrt(dx^2+dy^2+(dz+g)^2));
 theta_d = atan((dx*cos(psi)+dy*sin(psi))/(dz+g));

e_phi = phi_d - phi;
e_theta = theta_d - theta;

%Ommit D influence for the first loop
if(e_phi_old == -1)
    e_phi_old = e_phi;
    e_theta_old = e_theta;
    e_psi_old = e_psi;
end

%Compute values of desired velocities
 phi_d_p = (e_phi-e_phi_old)/dT;
 psi_d_p = (e_psi - e_psi_old)/dT;
 theta_d_p = (e_theta-e_theta_old)/dT;

%Add actual values of e_x to buffer
e_phi_old = e_phi;
e_theta_old = e_theta;
e_psi_old = e_psi;


%PD Controller
 T = m*(dx*(sin(theta)*cos(psi)*cos(phi)+sin(psi)+sin(phi)) + dy*(sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi))   + (dz+g)*cos(theta)*cos(phi));
 tau_phi = ( kphi_d*(phi_d_p-phi_p) + kphi_p*(e_phi))*Ixx; 
 tau_psi = ( kpsi_d*(psi_d_p-psi_p) + kpsi_p*(e_psi))*Izz; 
 tau_theta = ( ktheta_d*(theta_d_p-theta_p) + ktheta_p*(e_theta))*Iyy;

output = [T tau_phi tau_psi tau_theta phi_d theta_d psi_d];

end

