function  Rotate_3d = Angles( angle_vector )

fi = angle_vector(1);   % fi - roll angle
theta = angle_vector(2);  % psi - yaw angle
psi = angle_vector(3);% theta - pitch angle

R = [cos(psi)*cos(theta) (cos(psi)*sin(theta)*sin(fi) - sin(psi)*cos(fi)) (cos(psi)*sin(theta)*cos(fi) + sin(psi)*sin(fi));
    sin(psi)*cos(theta) (sin(psi)*sin(theta)*sin(fi) + cos(psi)*cos(fi)) (sin(psi)*sin(theta)*cos(fi) - cos(psi)*sin(fi));
    -sin(theta) cos(theta)*sin(fi) cos(theta)*cos(fi)];     % Rotation matrix

Rotate_3d = R;
end

