function [q] = Inverse(end_effector,l_1,l_2,leg)
%INVERSE Summary of this function goes here
% Calculating needed displacement for joints to reach the end effector
% positions

%   Detailed explanation goes here
% INPUTS: end_effector - position of end effector = [x,y,z,angle_x,angle_y,angle_z]
% l_1,l_2 - lengths of first and second link, in my solution they are equal
% for all legs
% leg - name of leg 'x' or 'y' or 'z' for calculating the inverse kinematics, because tripteron has 3 legs

% OUTPUTS: q - array of needed displacements for each joint(1 - 4) to
% obtain given end effector position

xx = end_effector(1);
yy = end_effector(2);
zz = end_effector(3);
angle_x = end_effector(4);
angle_y = end_effector(5);
angle_z = end_effector(6);


% elbow down
if leg=='x'
    
%     taking into account that frame of leg 'x' is shifted
    yy = yy-0.5;
    zz = zz+0.25;
    
%     inverse kinematic solution for elbow down
    q_11 = xx;
    q_33 = acos((yy^2+zz^2-l_1^2-l_2^2)/(2*l_1*l_2));
    q_22 = -atan2((l_2*sin(q_33)),(l_1+l_2*cos(q_33))) + atan2(zz,yy);

    q_44 = angle_x - q_22 - q_33;
    
end
    
% elbow down
if leg=='y'

    %     taking into account that frame of leg 'y' is shifted
    zz = zz-0.25;
    yy = yy-0.25;
    
    q_11 = yy;
    q_33 = acos((zz^2+xx^2-l_1^2-l_2^2)/(2*l_1*l_2));

    q_22 = -atan2(l_2*sin(q_33),l_1+l_2*cos(q_33)) + atan2(xx,zz);

    q_44 = angle_y - q_22 - q_33;
    
end

% elbow down
if leg=='z'
    
%     frame of leg 'z' is the same as global frame
%     inverse kinematic for elbow down
    q_11 = zz;
    q_33 = acos((xx^2 + yy^2 - l_1^2 - l_2^2)/(2 * l_1 * l_2));
    q_22 = -atan2(l_2*sin(q_33),l_1+l_2*cos(q_33)) + atan2(yy,xx);

    q_44 = angle_z - q_22 - q_33;
    
end

q = [q_11,q_22,q_33,q_44];


