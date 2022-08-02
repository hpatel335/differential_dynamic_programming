# Differential Dynamic Programming

This project details the formulation of Differential Dynamic Programming in discrete time. Implementation of the algorithms for various systems including an inverted pendulum and a cart-pole have also been documented in this report. 

## Overview

The optimal control problem is defined as: 

$$
    V(x(t_0), t_0) = \underset{u}{min}\left[\phi(\mathbf{x}(t_f), t_f) + \int_{t_0}^{t_N} l(\mathbf{x, u}, t)dt \right]
$$

where x and u represent the state and the controls with dimensionalities n and m respectively.  The general dynamics of the system are represented as follows: 

$$
    \frac{dx}{dt} = f(\mathbf{x, u}, t)
$$

The Bellman principle in discrete time is given as: 

$$
V(\mathbf{x}(t_k),t_k)=\underset{u}{min}\left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

where L and V represent the running cost in discrete time and value function at $t= t_k$. The state value function is defined as follows: 

$$
Q(\mathbf{x}(t_k), \mathbf{u}(t_k)) =  \left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

## DDP Implimentation
The steps needed to derive the Differential Dynamic Programming scheme are detailed in this [source](https://ieeexplore.ieee.org/document/5530971).

This section details the implementation of the DDP scheme derived in the previous section. DDP will be applied to an inverted pendulum, and a cart pole system. In general each implementation will utilize the  pseudocode detailed in the table below: 

| Steps | Description | 
| ----- | ----------- | 
| 1 | Given the nominal states and controls, determine the linearized dynamics of the system  |
| 2 | Calculate the second order expansion of the state value function|
| 3 | Back-propagate the value function, its gradient and its hessian |
| 4 |  Update the controls using the optimal control correction | 
| 5 | Calculate the new optimal trajectory by applying the new controls to the system dynamics | 
| 6 | Set the new nominal trajectory and control as the calculated optimal trajectory and control| 
| 7 | Check for convergence, repeat steps 1-7 until converged | 


#### Inverted Pendulum 
This section describes the implementation of our DDP derivation to an inverted pendulum system. It's dynamics are given with the equation: 

$$
I\ddot{\theta}+b\dot{\theta}+mglsin(\Theta)=u
$$ 

\noindent Where $I=ml^2,\,g=9.81\,m/s^2$, m is the mass, l is the length, b is damping, and f is the control. This equation is then converted to state space form:\\

$$
F(x,u,t)=\frac{dx}{dt}=\left[ \begin{array}{c} \dot{x_1} \\ \dot{x_2} \end{array}\right] = \left[ \begin{array}{c} x_2 \\ \frac{u}{ml^2} - \frac{g}{l}sin(x_1)-\frac{b}{ml^2}x_2\end{array}\right]=\left [\begin{array}{c} f(x_1,x_2) \\ g(x_1,x_2)\end{array}\right ]
$$

Where $x_1=\theta$ and $x_2=\dot{\theta}$. $\frac{dx}{dt}$ is then used in step 5 of the DDP Algorithm as described the table above with the relation $x(k+1)=x(k)+\frac{dx}{dt}(k)dt$. In order to find $\triangledown_xF$ and $\triangledown_uF$, the Hessian of the dynamics matrix above must be found with respect to x and u. 


