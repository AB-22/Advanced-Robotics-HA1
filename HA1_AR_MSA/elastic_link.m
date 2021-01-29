function [W,t] = elastic_link(length,q,leg) %q_x 
%ELASTIC_LINK Summary of this function goes here
%   Detailed explanation goes here


E = 70e9; %Young's modulus
G = 25.5e9; %shear modulus
d = 10*10e-3;
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;

if leg =='x'
    
%     Q1 - rotation matrix for changing the orientation as in global 
    R_x1 = Rx(q);
    R_x1 = R_x1(1:3,1:3);
    Q1 = zeros(12);
    Q1(1:3,1:3) = R_x1;
    Q1(4:6,4:6) = R_x1;
    Q1(7:9,7:9) = R_x1;
    Q1(10:12,10:12) = R_x1;
%     Q2 - inverse of Q1
    Q2 = zeros(12);
    Q2(1:3,1:3) = R_x1';
    Q2(4:6,4:6) = R_x1';
    Q2(7:9,7:9) = R_x1';
    Q2(10:12,10:12) = R_x1';

%     stiffness matrix for cylindrical object
    K = k_cylinder(E, G, d, length(1), S, Iy, Iz);

%     here i am calculating the parameters 12x12 matrix K for elastic link
%     according to the algorithm in KLIMCHIK's presentation for MSA
    K22=K;
%     length(1) - link length, others should be zero(in my solution they are very small for numerical stability)
    l_skew = [0,-length(3),length(2);
              length(3),0,-length(1);
              -length(2),length(1),0];

    
    transform = -[eye(3),zeros(3);
                  l_skew',eye(3)];
    K12 = transform * K22;


    roty = Ry(pi);
    full_roty = [roty(1:3,1:3),zeros(3); 
                 zeros(3),roty(1:3,1:3)];

    K11 = full_roty' * K * full_roty;

    transform = -[eye(3),zeros(3);
                  l_skew,eye(3)];
    K21 = transform * K11;
end
        
if leg =='y'
%     Q1 - rotation matrix for changing the orientation as in global 
    R_y1 = Ry(q);
    R_y1 = R_y1(1:3,1:3);
    Q1 = zeros(12);
    Q1(1:3,1:3) = R_y1;
    Q1(4:6,4:6) = R_y1;
    Q1(7:9,7:9) = R_y1;
    Q1(10:12,10:12) = R_y1;
%     Q2 - inverse of Q1
    Q2 = zeros(12);
    Q2(1:3,1:3) = R_y1';
    Q2(4:6,4:6) = R_y1';
    Q2(7:9,7:9) = R_y1';
    Q2(10:12,10:12) = R_y1';

%     stiffness for cylindrical object
    K = k_cylinder(E, G, d, length(1), S, Iy, Iz);

%     calculating 12x12 K matrix for elastic link as in KLIMCHIK's
%     presentation
    K22=K;
    l_skew = [0,-length(3),length(2);
              length(3),0,-length(1);
              -length(2),length(1),0];

    transform = -[eye(3),zeros(3);
                  l_skew',eye(3)];
    K12 = transform * K22;


    rotz = Rz(pi);
    full_rotz = [rotz(1:3,1:3),zeros(3); %may be here mistake!!!!!!
                 zeros(3),rotz(1:3,1:3)];

    K11 = full_rotz' * K * full_rotz;

    transform = -[eye(3),zeros(3);
                  l_skew,eye(3)];
    K21 = transform * K11;
    
end

if leg =='z'
%     Q1 - rotation matrix for moving
    R_z1 = Rz(q);
    R_z1 = R_z1(1:3,1:3);
    Q1 = zeros(12);
    Q1(1:3,1:3) = R_z1;
    Q1(4:6,4:6) = R_z1;
    Q1(7:9,7:9) = R_z1;
    Q1(10:12,10:12) = R_z1;
    
    Q2 = zeros(12);
    Q2(1:3,1:3) = R_z1';
    Q2(4:6,4:6) = R_z1';
    Q2(7:9,7:9) = R_z1';
    Q2(10:12,10:12) = R_z1';

    
    K = k_cylinder(E, G, d, length(1), S, Iy, Iz);

    K22=K;
    l_skew = [1e-8,-length(3),length(2);
              length(3),1e-8,-length(1);
              -length(2),length(1),1e-8];

    transform = -[eye(3),zeros(3);
                  l_skew',eye(3)];
    K12 = transform * K22;


    roty = Ry(pi);
    full_roty = [roty(1:3,1:3),zeros(3); %may be here mistake!!!!!!
                 zeros(3),roty(1:3,1:3)];

    K11 = full_roty' * K * full_roty;

    transform = -[eye(3),zeros(3);
                  l_skew,eye(3)];
    K21 = transform * K11;
end


W = [-eye(12)];
t = [K11,K12;
     K21,K22];             

% transforming to global orientation
t = Q1*t*Q2;


end

