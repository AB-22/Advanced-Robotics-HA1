function [lambda_p,lambda_r] = lambda_p(leg)
%LAMBDA_P Summary of this function goes here
% Calculating diagonal matrices lambda_p and lambda_r for passive joints
% element of lambda_p which is equal to 1 means axis of rotation of passive
% joint

%   Detailed explanation goes here
% INPUTS:
% chosen leg - all passive joint of the same leg are only rotating in the
% same axis
% OUTPUTS:
% diagonal matrices lambda_p and lambda_r


if leg=='x'
    lambda_p = [0 0 0 1 0 0];
    lambda_r = [1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
end
  
if leg=='y'
    lambda_p = [0 0 0 0 1 0];
    lambda_r = [1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 0 1];
end

if leg=='z'
    lambda_p = [0 0 0 0 0 1];
    lambda_r = [1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0];
end
end

