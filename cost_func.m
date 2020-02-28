function  cost   = cost_func( U, K, x_i, x_g, dt )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here

cost = 0; %initial

R = [4 0;0 20] ;
Q = [10 0 0 0;0 10 0 0;0 0 3 0;0 0 0 3];
Qf = [500 0 0 0;0 500 0 0;0 0 1 0;0 0 0 1];
for i = 1:K
   
    if i==1
        X(:,i) = car_robot_dynamics(x_i, U(:,i),dt);%calculate new state
    
    else
        X(:,i) = car_robot_dynamics(X(:,i-1), U(:,i),dt);%calculate new state
        
    end
    %{
    if (X(1,i)-3.5)^2 + (X(2,i)-2)^2 -(.5)^2 < 0
        obstacle_cost = 10000000000;

    elseif (X(1,i)-6)^2 + (X(2,i)-3)^2 -(0.5)^2 < 0
        obstacle_cost = 10000000000;
    elseif (X(1,i)-3)^2 + (X(2,i)-4)^2 -(0.5)^2 < 0
        obstacle_cost = 10000000000;

    else
        obstacle_cost = 0 ;
    end
    %}
    if i~=K
        cost = cost + U(:,i)'*R*U(:,i) + (x_g - X(:,i))'*Q*(x_g - X(:,i)); %+ obstacle_cost;
    else
        cost = cost + U(:,i)'*R*U(:,i) + (x_g - X(:,i))'*Qf*(x_g - X(:,i));
   
end

end

