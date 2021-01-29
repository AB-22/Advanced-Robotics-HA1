function [W,t] = elastic_joint(leg)
%ELASTIC_JOINT Summary of this function goes here
% calculating aggregation MSA model for elastic joint

%   Detailed explanation goes here
% INPUT:
% leg of tripteron robot
% OUTPUT:
% Aggregation MSA Model for elastic joint


k = 1e-6; %actuator stiffness as it was given in example

% lambda_e - diagonal matrix with axis of translation of elastic joint = 1 
% lambda_r - diagonal matrix with axis of all other elements =1 
[lambd_e,lambd_r] = lambda_e(leg); 

% Aggregation MSA model equation for elastic joint
W = [zeros(5,6),zeros(5,6); %this is the one which is multiplied to W
         eye(6),     eye(6);
         lambd_e, zeros(1,6)];
     
t =  [lambd_r,-lambd_r; %this is the one which is multiplied to delta t
      zeros(6),     zeros(6);
      k*lambd_e, -k*lambd_e];
   
    


end

