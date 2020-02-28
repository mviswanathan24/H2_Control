function [U] = bound(U)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if U(1)>4
    U(1)=4;
elseif U(1)< -4
    U(1)= -4;
end 
if U(2) > pi/3
    U(2) = pi/3;
elseif U(2)< -pi/3
    U(2) = -pi/3;

end

