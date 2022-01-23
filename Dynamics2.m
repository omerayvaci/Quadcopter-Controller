function output = Dynamics( input )

global k b Ixx Iyy Izz Ir L m Ir_2 

%This part is considered in body frame, transformations between BCS
%(body coordinate system) and ECS (earth coordinate system) is included in file Dynamics_earth

w1 = input(1);      %Angular velocity of motor 1
w2 = input(2);      %Angular velocity of motor 2
w3 = input(3);      %Angular velocity of motor 3
w4 = input(4);      %Angular velocity of motor 4

phi_p_b = input(5);     %Angular velocity in roll in BCS
theta_p_b = input(6);   %Angular velocity in pitch in BCS
psi_p_b = input(7);     %Angular velocity in yaw in BCS

phi = input (9);
theta = input (10);
psi = input (11);

phi_p_b_hat = input (13);
theta_p_b_hat = input (14);
psi_p_b_hat = input (15);

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

F = k*(sign(w1)*w1^2 + sign(w2)*w2^2 + sign(w3)*w3^2 + sign(w4)*w4^2);          %Force in Z(body oriented) axis made by rotors
tauM = b*sign(w1)*w1^2 - b*sign(w2)*w2^2 + b*sign(w3)*w3^2 - b*sign(w4)*w4^2;   %Torque made by spinning propellers

tau_phi_b =  L*k*(-sign(w2)*w2^2+sign(w4)*w4^2);        %Torque on roll angle in BCS 
tau_theta_b = L*k*(-sign(w1)*w1^2 + sign(w3)*w3^2);     %Torque on yaw angle in BCS 
tau_psi_b = tauM;                                       %Torque on pitch angle in BCS

w_g = (sign(w1)*w1-sign(w2)*w2+sign(w3)*w3-sign(w4)*w4); %Gyroscopic torque

a_z_b   = F/m;      %Acceleration on Z axis in BCS

%Acceleration is due to Centripetal forces + Gyroscopic forces + External torque
phi_bis_b = (Iyy-Izz)*theta_p_b*psi_p_b/Ixx - Ir_2*(theta_p_b/Ixx)*w_g + tau_phi_b/Ixx;     % angular acceleration in roll in BCS 
theta_bis_b = (Izz-Ixx)*phi_p_b*psi_p_b/Iyy + Ir_2*(phi_p_b/Iyy)*w_g   + tau_theta_b/Iyy;   % angular acceleration in pitch in BCS
psi_bis_b = (Ixx-Iyy)*theta_p_b*phi_p_b/Izz +           0            + tau_psi_b/Izz;       % angular acceleration in yaw in BCS

phi_p_diff = (phi - phi_p_b_hat);
theta_p_diff = (theta - theta_p_b_hat);
psi_p_diff = (psi - psi_p_b_hat);

phi_bis_b_1 = phi_bis_b + phi_p_diff;
theta_bis_b_1 = theta_bis_b + theta_p_diff;
psi_bis_b_1 = psi_bis_b + psi_p_diff;

output = [a_z_b phi_bis_b_1 theta_bis_b_1 psi_bis_b_1];     %Accelerations in BCS                                      

end

