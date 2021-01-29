function [W,t] = passive_joint(leg)
%PASSIVE_JOINT Summary of this function goes here
% calculating aggregation MSA model for passive joint

%   Detailed explanation goes here
% INPUTS:
% chosen leg
% OUTPUTS:
% aggregation MSA model for passive joint - part which is multiplied by W
% and part which is multiplied by delta t



[lambd_p,lambd_r] = lambda_p(leg);

W = [zeros(5,6),zeros(5,6);  %multiplied by W
     lambd_r,lambd_r;
     lambd_p,zeros(1,6);
     zeros(1,6),lambd_p];

                 
t = [lambd_r,-lambd_r;  %multiplited by delta t
     zeros(5,6),zeros(5,6);
     zeros(1,6),zeros(1,6);
     zeros(1,6),zeros(1,6)];


end

