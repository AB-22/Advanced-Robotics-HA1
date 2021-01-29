function [lambda_e,lambda_r] = lambda_e(leg)
%LAMBDA_E Summary of this function goes here
% Calculating diagonal matrices lambda_e and lambda_r for elastic joint,
% all elastic joints are only translation -> only first 3 elements of
% lambda_e will be equal 1 depending on leg

%   Detailed explanation goes here
% INPUTS:
% leg - 'x' 'y' or 'z' leg of tripteron robot
% OUTPUTS:
% lambda_e and lambda_r matrices


% all first joints are tranlational along chosen axis -> so we change one
% of first elements depending on axis of rotation

if leg=='x'
    lambda_e = [1 0 0 0 0 0];
    lambda_r = [0 1 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
end
  
if leg=='y'
    lambda_e = [0 1 0 0 0 0];
    lambda_r = [1 0 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
end

if leg=='z'
    lambda_e = [0 0 1 0 0 0];
    lambda_r = [1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];
end
        
end