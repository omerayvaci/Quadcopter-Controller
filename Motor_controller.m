function [ output ] = Motor_controller( input )

%This part is translate the informations of desired forces to
%informations of desired motor velocities

global L k b

F = input(1);           %Force in z axis
tau_phi = input(2);     %Torque in roll
tau_theta = input(4);   %Torque in pitch
tau_psi = input(3);     %Torque in yaw

w1_sq = F/(4*k) - tau_theta/(2*k*L) + tau_psi/(4*b); %Motor 1 speed  
w2_sq = F/(4*k) - tau_phi/(2*k*L) - tau_psi/(4*b);   %Motor 2 speed
w3_sq = F/(4*k) + tau_theta/(2*k*L) + tau_psi/(4*b); %Motor 3 speed
w4_sq = F/(4*k) + tau_phi/(2*k*L) - tau_psi/(4*b);   %Motor 4 speed

if(w1_sq < 0)
    w1_sq = 0;
end

if(w2_sq < 0)
    w2_sq = 0;
end

if(w3_sq < 0)
    w3_sq = 0;
end

if(w4_sq < 0)
    w4_sq = 0;
end

w1 = sqrt(abs(w1_sq));
w2 = sqrt(abs(w2_sq));
w3 = sqrt(abs(w3_sq));
w4 = sqrt(abs(w4_sq));

 if(w1 > 3000)
     w1 = 3000;
 end
 
 if(w2 > 3000)
     w2 = 3000;
 end
 
 if(w3 > 3000)
     w3 = 3000;
 end
 
 if(w4 > 3000)
     w4 = 3000;
 end

        
output = [w1 w2 w3 w4];

end

