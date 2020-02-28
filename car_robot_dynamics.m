function [ X_new ] = car_robot_dynamics( X_prev, U,dt )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here]

L = .01;
X_new = X_prev + dt*[U(1)*(cos(X_prev(3))); U(1)*(sin(X_prev(3))) ; U(1)*tan(X_prev(4))/L; U(2)];


end

