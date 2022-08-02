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

This section details the implementation of the DDP scheme derived in the previous section. DDP will be applied to an inverted pendulum, and a cart pole system. In general each implementation will utilize the  pseudocode detailed in Table \ref{tab:pseudocode}.

| Steps | Description | 
| ----- | ----------- | 
| 1 | Given the nominal states and controls, determine the linearized dynamics of the system  |
| 2 | Calculate the second order expansion of the state value function|
| 3 | Back-propagate the value function, its gradient and its hessian |
| 4 |  Update the controls using the optimal control correction | 
| 5 | Calculate the new optimal trajectory by applying the new controls to the system dynamics | 
| 6 | Set the new nominal trajectory and control as the calculated optimal trajectory and control| 
| 7 | Check for convergence, repeat steps 1-7 until converged | 


