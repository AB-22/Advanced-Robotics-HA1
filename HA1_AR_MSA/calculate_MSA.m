function [Kc] = calculate_MSA(leg,end_effector)
%CALCULATE_MSA Summary of this function goes here
% Calculating stiffness matrix 

%   Detailed explanation goes here
% INPUTS:
% leg - chosen leg of tripteron 'x' or 'y' or 'z'
% end efffector - end effector position [x,y,z,angle_x,angle_y,angle_z]

% matrix for rigid base
rigid_base_W = [zeros(6)]; %part which will be multiplied by W
rigid_base_t = [eye(6)]; %part which will be multiplied by delta t


% MSA for ELASTIC JOINT #1(W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_joint_1_W,elastic_joint_1_t] = elastic_joint(leg);


% MSA for PASSIVE JOINT #2(W - part which will be multiplied by W, t- part which will be multiplied by delta t)              
[passive_joint_2_W,passive_joint_2_t] = passive_joint(leg);

% MSA for PASSIVE JOINT #3(W - part which will be multiplied by W, t- part which will be multiplied by delta t)

[passive_joint_3_W,passive_joint_3_t] = passive_joint(leg);
                 
% MSA for PASSIVE JOINT #4(W - part which will be multiplied by W, t- part which will be multiplied by delta t)

[passive_joint_4_W,passive_joint_4_t] = passive_joint(leg);             

%parameters of rigid links
l_1r=[0,10e-4,10e-4];
l_2r=[0.1,10e-4,10e-4];

%MSA for Rigid Link #1
[rigid_link_1_W,rigid_link_1_t]=rigid_link(l_1r);
%MSA for Rigid Link #2  (rigid platform as a rigid link)
[rigid_link_2_W,rigid_link_2_t]=rigid_link(l_2r);


% parameters of elastic links
l_1 = [1,10e-4,10e-4];
l_2 = [1,10e-4,10e-4];

% Inverse kinematics to obtain positions for each joint
q_x = Inverse(end_effector,l_1(1),l_2(1),leg);

%MSA for ELASTIC LINK #1 (W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_link_1_W,elastic_link_1_t] = elastic_link(l_1,q_x(2),leg);

%MSA for ELASTIC LINK #2 (W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_link_2_W,elastic_link_2_t] = elastic_link(l_2,q_x(2),leg);


% here comes the aggregation
      
Aggregated = zeros(108,108);

% here the part with W
% Aggregated part for multiplying with W
Aggregated(1:6,1:6) = rigid_base_W;%0,1
Aggregated(7:18,1:12) = elastic_joint_1_W;%1,2
Aggregated(19:30,7:18)= rigid_link_1_W;%2,3
Aggregated(31:42,13:24) = passive_joint_2_W;%3,4
Aggregated(43:54,19:30) = elastic_link_1_W;%4,5
Aggregated(55:66,25:36) = passive_joint_3_W;%5,6
Aggregated(67:78,31:42) = elastic_link_2_W;%6,7
Aggregated(79:90,37:48) = passive_joint_4_W;%7,8
Aggregated(91:102,43:54) = rigid_link_2_W; %8,e
Aggregated(103:108,49:54)=-eye(6);%Denoting external loading
%  Aggregated part for multiplying with delta t
Aggregated(1:6,55:60) = rigid_base_t;
Aggregated(7:18,55:66) = elastic_joint_1_t;
Aggregated(19:30,61:72)= rigid_link_1_t;%2,3
Aggregated(31:42,67:78) = passive_joint_2_t;%3,4
Aggregated(43:54,73:84) = elastic_link_1_t;%4,5
Aggregated(55:66,79:90) = passive_joint_3_t;%5,6
Aggregated(67:78,85:96) = elastic_link_2_t;%6,7
Aggregated(79:90,91:102) = passive_joint_4_t;%7,8
Aggregated(91:102,97:108) = rigid_link_2_t; %8,e
Aggregated(103:108,103:108)=zeros(6); %Denoting external loading
% now We should separate this big matrix into A,B,C,D

A = Aggregated(1:102,1:102);
B = Aggregated(1:102,103:108);
C = Aggregated(103:108,1:102);
D = Aggregated(103:108,103:108);

% obtaining stiffnes matrix as in KLIMCHIK'S presentation
Kc = D - C * inv(A) * B;


end
