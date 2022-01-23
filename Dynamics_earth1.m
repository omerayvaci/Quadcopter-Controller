function [ output ] = Dynamics_earth( input )

global g

a_z_b = input(1);       %Linear acceleration in Z axis in BCS
phi_bis_b = input(2);   %Angular acceleration in roll in BCS
theta_bis_b = input(3); %Angular acceleration in pitch in BCS
psi_bis_b = input(4);   %Angular acceleration in yaw in BCS

phi = atan2(sin(input(5)),cos(input(5)));      %Actual roll angle in earth frame
theta = atan2(sin(input(6)),cos(input(6)));    %Actual pitch angle in earth frame
psi = atan2(sin(input(7)),cos(input(7)));      %Actual yaw angle in earth frame 

phi_p = input(8);      %Actual velocity in roll in earth frame
theta_p = input(9);    %Actual velocity in pitch in earth frame
psi_p = input(10);     %Actual velocity in yaw in earth frame

phi_p_b = input(11);    %Angular velocity in BCS (roll)
theta_p_b = input(12);  %Angular velocity in BCS (pitch)
psi_p_b = input(13);    %Angular velocity in BCS (yaw)

W = [1 0 -sin(theta); 
    0 cos(phi) cos(theta)*sin(phi); 
    0 -sin(phi) cos(theta)*cos(phi)];

%inv_W_p is time derivative of inverse of W matrix

inv_W_p = [0 (phi_p*cos(phi)*tan(theta) + theta_p*sin(phi)/(cos(theta)^2)) (-phi_p*sin(phi)*cos(theta) + theta_p*cos(phi)/(cos(theta)^2));
           0 -phi_p*sin(phi) -phi_p*cos(phi);
           0 (phi_p*cos(phi)/cos(theta)+theta_p*sin(phi)*tan(theta)/cos(theta)) (-phi_p*sin(phi)/cos(theta) + theta_p*cos(phi)*tan(theta)/cos(theta))];

%One of phi_p in second element in last row was replaced by theta_p
        
earth_angular_acc = inv_W_p*(W*[phi_p theta_p psi_p]') + W\[phi_bis_b theta_bis_b psi_bis_b]' ; 

phi_bis = earth_angular_acc(1);
theta_bis = earth_angular_acc(2);
psi_bis = earth_angular_acc(3);

earth_linear_acc = -g*[0 0 1]' + Rotate_3d([phi theta psi])*[0 0 a_z_b]';

x_bis = earth_linear_acc(1);
y_bis = earth_linear_acc(2);
z_bis = earth_linear_acc(3);

output = [x_bis y_bis z_bis phi_bis theta_bis psi_bis]; 

end

