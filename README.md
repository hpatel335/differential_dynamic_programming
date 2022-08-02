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
