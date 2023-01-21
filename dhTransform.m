% dhTransform: Returns the homogenous transform corresponding to the provided DH parameters
% for a link. 
%
% [ H ] = dhTransform(a, d, alpha, theta)) for the given link and parametrs of the robot
% the function returns a homogenous transformation matrix of 4-by-4. 
%
% output1 = 4-by-4 homogenous transformation matrix   
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = a is the link length (m/cm/mm)
% input2 = d is the link offset (m/cm/mm) both d and a must have same unit
% input3 = alpha is the link twist in radians
% input4 = theta is the joint angle in radians
%
% Abhilesh Borode
% 10830440
% MEGN 544 
% 18th Nov 2018

function [ H ] = dhTransform(a, d, alpha, theta)
H=[cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),a*cosd(theta);
    sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sin(alpha),a*sind(theta);
    0,sind(alpha),cosd(alpha),d;
    0,0,0,1] % homogenous transformation matrix in terms of joint and link parameters
end
