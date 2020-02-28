function [ X_new ] = dynamics_uncertain( X_prev, U,dt,w)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here]

L = .01;
U(1) = U(1)+ 4*w(1);
U(2) = U(2)+ pi/3*w(2);

X_new = X_prev + dt*[U(1)*(cos(X_prev(3))); 
                U(1)*(sin(X_prev(3))) ; 
                U(1)*tan(X_prev(4))/L; U(2)];%+ [w(1);w(2);0;0];


end

