clear; clc;clf;

%% policy_iteration (open loop trajectory)

dt = .1; %time step
K = 15; %time horizon
L = .01;
x_i = [0; 0; 0; 0];% robot initial position [x y theta steering_angle]
x_g = [2; 2; 0; 0];% robot goal

%{
U_lb = [-4*ones(1,K);-(pi/3)*ones(1,K)];%lower bound on control inputs
U_ub = [4*ones(1,K);(pi/3)*ones(1,K)];%upper bound
%U = zeros(2,K)
f =@(U) cost_func(U, K, x_i, x_g, dt)

options = optimset('Algorithm', 'interior-point', 'MaxIter', 10000, 'MaxFunEvals', 100000);
[U_opti, fval] = fmincon(f, zeros(2, K), [], [], [], [], U_lb, U_ub,@(U)constraints(U, K, x_i, x_g, dt), options)

for i = 1:K
   
    if i==1
        X(:,i) = car_robot_dynamics(x_i, U_opti(:,i),dt);%calculate new state
    
    else
        X(:,i) = car_robot_dynamics(X(:,i-1), U_opti(:,i),dt);%calculate new state
        
    end
end

%{
hold on
th = 0:pi/50:2*pi;

xunit = 0.5* cos(th) + 3.5;
yunit = 0.5* sin(th) + 2;
plot(xunit, yunit);

xunit =  0.5* cos(th) + 6;
yunit = 0.5* sin(th) + 3;
plot(xunit, yunit);
xunit =  0.5* cos(th) + 3;
yunit = 0.5* sin(th) + 4;
plot(xunit, yunit);

hold off 
%}
figure(1)
plot(X(1,:),X(2,:))
%}
%% H2 controller synthesis 
load('optimal_taj.mat')

for iter=1:K
    X_l = X(:,iter); %taking ith time step as linearisation 
    U_l = U_opti(:,iter);

    nx = 4; nu = 2; nw =1; nz = 6;

    A = [1 0 -U_l(1)*sin(X_l(3))*dt 0;
        0  1 U_l(1)*cos(X_l(3))*dt 0;
        0 0 1 U_l(1)*(sec(X_l(4))^2)*dt/L;
        0 0 0 1];
    A = (A-eye(4))*(1/dt);
    Bu = [cos(X_l(3)) 0;
        sin(X_l(3)) 0;
        tan(X_l(4))/L 0;
        0 1];
 
    Bw = [cos(X_l(3)) 0;
        sin(X_l(3)) 0;
        tan(X_l(4))/L 0;
        0 1];

    Cz = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 2;10^-8*ones(2,4)]; %z is the parameter we need to keep small

    Du = [0 0;0 0;0 0;0 0;1 0;0 50]; %cost on control input

    cvx_begin sdp

    variable P(nx,nx) symmetric
    variable W(nz,nz) symmetric
    variable Z(nu,nx)

    [A Bu]*[P;Z] + [P Z']*[A';Bu'] + Bw*Bw' <= (10^-6)*ones(4,4)
    [W (Cz*P + Du*Z); (Cz*P + Du*Z)' P] >= (10^-6)*ones(10,10)
    minimize trace(W)
    cvx_end

    h2K(:,:,iter) = Z*inv(P);
end

%% Run_simulation
clear X_a X_unc;
for i = 1:K
    if i==1
        w = 0.001*randn(2,1);
        X_a(:,i) = dynamics_uncertain(x_i, U_opti(:,i),dt,w);%calculate new state
        X_unc(:,i) = dynamics_uncertain(x_i, U_opti(:,i),dt,w);
    else
        error = X(:,i-1) - X_a(:,i-1);
        error(3) = atan2(sin(error(3)),cos(error(3)));
        error(4) = atan2(sin(error(4)),cos(error(4)));
        disp('error:');disp(error);
        U_h2 = U_opti(:,i) - h2K(:,:,i)*error;
        U_h2 = bound(U_h2);
        disp('del_U:');disp(h2K(:,:,i)*error)
        w = 0.01*randn(2,1);
        X_a(:,i) = dynamics_uncertain(X_a(:,i-1),U_h2,dt,w);%calculate actual state
        X_unc(:,i) = dynamics_uncertain(X_unc(:,i-1),U_opti(:,i),dt,w);
    end

end

X = [x_i X];

figure(1)
plot(X(1,:),X(2,:),'r')
hold on 
X_a = [x_i X_a];
X_unc = [x_i X_unc];
plot(X_a(1,:),X_a(2,:),'g')
plot(X_unc(1,:),X_unc(2,:),'b')
xlabel('x position')
ylabel('y position')
title('X vs Y')
legend('nominal','H2 controlled','uncontrolled')

A1=A+Bu*h2K(:,:,2)
B1=Bw
C2=eye(4)
D2=zeros(4,2)
C1=Cz+Du*h2K(:,:,2)
D1=zeros(6,2)
sys=ss(A1,B1,C2,D2)

impulse(sys)