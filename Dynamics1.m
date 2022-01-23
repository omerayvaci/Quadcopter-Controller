function output = Dynamics( input )

global k b Ixx Iyy Izz Ir L m Ir_2 

%This part is considered in body frame, transformations between BCS
%(body coordinate system) and ECS (earth coordinate system) is included in file Dynamics_earth

w1 = input(1);      %angular velocity of motor 1
w2 = input(2);      %angular velocity of motor 2
w3 = input(3);      %angular velocity of motor 3
w4 = input(4);      %angular velocity of motor 4

phi_p_b = input(5);     % angular velocity in roll in BCS
theta_p_b = input(6);   % angular velocity in pitch in BCS
psi_p_b = input(7);     % angular velocity in yaw in BCS

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

F = k*(sign(w1)*w1^2 + sign(w2)*w2^2 + sign(w3)*w3^2 + sign(w4)*w4^2);          % force in Z(body oriented) axis made by rotors
tauM = b*sign(w1)*w1^2 - b*sign(w2)*w2^2 + b*sign(w3)*w3^2 - b*sign(w4)*w4^2;   % torque made by spinning propellers 

tau_phi_b =  L*k*(-sign(w2)*w2^2+sign(w4)*w4^2);        % torque on roll angle in BCS 
tau_theta_b = L*k*(-sign(w1)*w1^2 + sign(w3)*w3^2);     % torque on yaw angle in BCS 
tau_psi_b = tauM;                                       % torque on pitch angle in BCS

w_g = (sign(w1)*w1-sign(w2)*w2+sign(w3)*w3-sign(w4)*w4); % gyroscopic torque

a_z_b   = F/m;   %Acceleration on Z axis in BCS

%Acceleration is due to centripetal forces + gyroscopic forces + external torque
phi_bis_b = (Iyy-Izz)*theta_p_b*psi_p_b/Ixx - Ir_2*(theta_p_b/Ixx)*w_g + tau_phi_b/Ixx;     % angular acceleration in roll in BCS 
theta_bis_b = (Izz-Ixx)*phi_p_b*psi_p_b/Iyy + Ir_2*(phi_p_b/Iyy)*w_g   + tau_theta_b/Iyy;   % angular acceleration in pitch in BCS
psi_bis_b = (Ixx-Iyy)*theta_p_b*phi_p_b/Izz +           0            + tau_psi_b/Izz;       % angular acceleration in yaw in BCS

output = [a_z_b phi_bis_b theta_bis_b psi_bis_b];     % accelerations in BCS                                      

end

