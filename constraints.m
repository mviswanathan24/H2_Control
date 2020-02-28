function [ c, ceq] = constraints(U, K, x_i, x_g, dt)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

c=[];
%{
for i = 1:K
    if i==1
        X(:,i) = car_robot_dynamics(x_i, U(:,i),dt);%calculate new state
    
    else
        X(:,i) = car_robot_dynamics(X(:,i-1), U(:,i),dt);%calculate new state
        
    end
    c = [c; X(1:2,i)-x_g(1:2)];

end
%}
dU1 = 2;
dU2 = pi/6;
Ai = eye(K); 
for i=1:K;
    if i~=K
        Ai(i,i+1) = -1;
    end
end
bi = ones(K,1);
c1 = Ai*U(1,:)' - dU1*bi;
c2 = -Ai*U(1,:)' - dU1*bi;
c3 = Ai*U(2,:)' - dU2*bi;
c4 = -Ai*U(2,:)' - dU2*bi;

c = [c1;c2;c3;c4];
ceq = 0;
end