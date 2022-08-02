function [x] = fnsimulate(xo,u_new,Horizon,dt,sigma)

global m ;
global g ;
global l ;
global I ;
global b ;

x = xo;

for k = 1:(Horizon-1)

    x(:,k+1) = x(:,k) + [x(2,k); (-m*g*l*sin(x(1,k))/I - b*x(2,k)/I + u_new(:,k)/I)]*dt ; 
end