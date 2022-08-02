function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt)

global m ;
global g ;
global l ;
global I ;
global b ; 

x1 = x(1,1);

A = [0 , 1 ;... 
    -(m*g*l/I)*cos(x1) , -b/I] ; 

B = [0;(1/I)] ; 

end 




