# ELEC 391 Self-Balancing Inverted Pendulum Robot

**Quick References:**

State Space Matrix:


$$
\begin{align}
\mathbf{A}=
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & \dfrac{-b_g}{M} & \dfrac{gm}{M} & 0\\
0 & 0 & 0 & 1\\
0 & \dfrac{-b_g}{lM} & \dfrac{g(M+m)}{lM} & 0\\
\end{bmatrix}
;& \quad 
\overline{\mathbf{x}} = 
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta} \\
\end{bmatrix}
\end{align}
$$