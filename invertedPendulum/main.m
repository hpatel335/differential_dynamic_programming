%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Differential Dynamic Programming               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc

global m ;
global g ;
global l ;
global I ;
global b ;

% Inverted Pendulum Model Parameter 
m       = 1         ; % kg 
g       = 9.81      ; % m/s^2 
l       = 1         ; % m 
I       = m*l^2     ; %kgm^2 
b       = 0.25      ; 

% Horizon 
Horizon = 500; % 1.5sec

% Number of Iterations
num_iter = 300 ; 

% Discretization
dt = 0.01;

% Weight in Final State:
Q_f         = zeros(2,2);
Q_f(1,1)    = 10000;
Q_f(2,2)    = 900;

% Weight in the Control:
R = 10 * eye(1);

% Initial Configuration:
xo          = zeros(2,1);
xo(1,1)     = 0;
xo(2,1)     = 0;

% Initial Control:
u_k     = zeros(1,Horizon-1);
du_k    = zeros(1,Horizon-1);


% Initial trajectory:
x_traj = zeros(2,Horizon);
 
% Target: 
p_target(1,1) = pi;
p_target(2,1) = 0;

% Learning Rate:c
gamma = 0.25 ;
 
for k = 1:num_iter

%------------------------------------------------> Linearization of the dynamics
%------------------------------------------------> Quadratic Approximations of the cost function 
for  j = 1:(Horizon-1)
    
     [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x_traj(:,j), u_k(:,j), j,R,dt);
      L(j) = dt * l0;
      L_x(:,j) = dt * l_x;
      L_xx(:,:,j) = dt * l_xx;
      L_u(:,j) = dt * l_u;
      L_uu(:,:,j) = dt * l_uu;
      L_ux(:,:,j) = dt * l_ux; 
    
    [dfx,dfu] = fnState_And_Control_Transition_Matrices(x_traj(:,j),u_k(:,j),du_k(:,j),dt);
    phi(:,:,j) = eye(2,2) + dfx * dt; %phi 
    B(:,:,j) = dfu * dt;  
end

%------------------------------------------------> Find the controls
Vxx(:,:,Horizon)    = Q_f;
Vx(:,Horizon)       = Q_f * (x_traj(:,Horizon) - p_target); 
V(Horizon)          = 0.5 * (x_traj(:,Horizon) - p_target)' * Q_f * (x_traj(:,Horizon) - p_target); 

%------------------------------------------------> Backpropagation of the Value Function
for j = (Horizon-1):-1:1
     Q_o    = L(j) + V(j+1) ; 
     Q_u    = L_u(:,j) + B(:,:,j)'*Vx(:, j+1) ; 
     Q_x    = L_x(:,j) + phi(:,:,j)'*Vx(:, j+1) ; 
     Q_xx   = L_xx(:,:,j) + phi(:,:,j)'*Vxx(:,:,j+1)*phi(:,:,j) ; 
     Q_uu   = L_uu(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j) ; 
     Q_ux   = L_ux(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*phi(:,:,j) ; 
     Q_xu   = Q_ux' ; 
     
     l_k(:,j)   = -inv(Q_uu)*Q_u ; 
     L_k(:,:,j) = -inv(Q_uu)*Q_ux ; 
     
     Vxx(:, :, j)   = Q_xx - Q_xu*inv(Q_uu)*Q_ux ; 
     Vx(:,j)        = Q_x - Q_xu*inv(Q_uu)*Q_u ; 
     V(:, j)        = Q_o - 0.5*Q_u'*inv(Q_uu)*Q_u ; 
     
end 

%----------------------------------------------> Find the controls
dx = zeros(2,1);
for i=1:(Horizon-1)    

   du = l_k(:,i) + L_k(:,:,i) * dx;
   dx = phi(:,:,i) * dx + B(:,:,i) * du;  
   u_new(:,i) = u_k(:,i) + gamma * du;
end

u_k = u_new;

%---------------------------------------------> Simulation of the Nonlinear System
[x_traj] = fnsimulate(xo,u_new,Horizon,dt,0);
[Cost(:,k)] =  fnCostComputation(x_traj,u_k,p_target,dt,Q_f,R);
x1(k,:) = x_traj(1,:);

fprintf('iLQG Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
end

time(1)=0;
for i= 2:Horizon
time(i) =time(i-1) + dt;  
end

subplot(1, 3, 1) 
hold on 
plot(time,x_traj(1,:),'linewidth',4);  
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
title('Theta', 'fontsize', 20) 
xlabel('Time in sec','fontsize',20)
hold off;
grid;

subplot(1, 3, 2) 
hold on 
plot(time,x_traj(2,:),'linewidth',4);  
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
title('Angular Velocity', 'fontsize', 20) 
xlabel('Time in sec','fontsize',20)
hold off;
grid;

subplot(1, 3, 3) 
hold on 
plot(Cost,'linewidth',2); 
title('Cost', 'fontsize', 20) 
xlabel('Time in sec','fontsize',20)
hold off;
grid on ;
