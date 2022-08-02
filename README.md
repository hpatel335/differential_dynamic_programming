# Differential Dynamic Programming

This project details the formulation of Differential Dynamic Programming in discrete time. Implementation of the algorithms for various systems including an inverted pendulum and a cart-pole have also been documented in this report. 

## Overview

The optimal control problem is defined as: 

$$
    V(x(t_0), t_0) = \underset{u}{min}\left[\phi(\mathbf{x}(t_f), t_f) + \int_{t_0}^{t_N} l(\mathbf{x, u}, t)dt \right]
$$

\noindent where x and u represent the state and the controls with dimensionalities n and m respectively.  The general dynamics of the system are represented as follows: 

$$
    \frac{dx}{dt} = f(\mathbf{x, u}, t)
$$

\noindent The Bellman principle in discrete time is given as: 

$$
V(\mathbf{x}(t_k),t_k)=\underset{u}{min}\left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

\noindent where L and V represent the running cost in discrete time and value function at $t= t_k$. The state value function is defined as follows: 

$$
Q(\mathbf{x}(t_k), \mathbf{u}(t_k)) =  \left[L(\mathbf{x}(t_k),\mathbf{u}(t_k),t_k) + V(\mathbf{x}(t_{k+1}),t_{k+1})\right]
$$

## DDP Derivation
The steps needed to derive the Differential Dynamic Programming scheme have been provided in this section. This includes the development of a linearization of the dynamics, expansion of the state value function up to the second order, and development of the optimal control law and the equation for the value function, as well as its gradient and hessian. The work contained in this section is based on this [source](https://ieeexplore.ieee.org/document/5530971).

### Linearization of Dynamics in Discrete Time

Let $x = \bar{x} + \delta x$ and $u = \bar{u} + \delta u$, then the dynamics can be represented in terms of their nominal and delta components. This is shown below. 

$$
\frac{dx}{dt} = F\left(\bar{x} + \delta x, \bar{u} + \delta u, t \right)
$$ 

\noindent By expanding the previous equation with a Taylor series and neglecting higher order terms, we can obtain the following: 

$$
\frac{dx}{dt} = F\left(\bar{x}, \bar{u} , t \right) + \frac{\partial F}{\partial x} \delta x + \frac{\partial F}{\partial u}\delta u
$$ 

\noindent By rearranging terms we can obtain the following expressions (note $F_x = \frac{\partial F}{\partial x}$ and $F_u = \frac{\partial F}{\partial u}$): 

$$
\frac{dx}{dt} - F\left(\bar{x}, \bar{u} , t \right) = F_x\delta x + F_u \delta u 
$$

$$
\frac{dx}{dt} - \frac{d\bar{x}}{dt} =  F_x\delta x + F_u \delta u  \\
$$

$$
\frac{d \delta x}{dt} =  F_x\delta x + F_u \delta u 
$$

\noindent It can then be shown that: 

$$ 
\frac{d \delta x}{dt} = \frac{\delta x(t_{k+1}) - \delta x(t_k)}{dt} = F_x\delta x + F_u \delta u
$$ 

$$ 
{\delta x(t_{k+1})} = \delta x(t_k) + F_x dt \delta x(t_k) + F_u dt \delta u(t_k) 
$$ 

$$
{\delta x(t_{k+1})} = \left(I_{n \times n} + F_x dt \right)\delta x(t_k) + F_u dt \delta u(t_k) 
$$ 

\noindent Letting $\phi(t_k) = \left(I_{n \times n} + F_x dt \right)$ and $B(t_k) = F_u dt$, we can show the \textbf{linearized version of the dynamics} in the following form: 

$$
{\delta x(t_{k+1})} = \phi(t_k)\delta x(t_k) + B(t_k)\delta u(t_k) 
$$
