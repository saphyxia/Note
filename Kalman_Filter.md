# Kalman Filter

## Recursive Processing

$$
\begin{array}{m} 
\hat{X_k} = \hat{X_{k-1}} + K_k * (Z_k - \hat{X_{k-1}}) \\
K_k:kalman Gain
\end{array}
$$

* $e_{EST}$: Estimate Error 
* $e_{MEA}$: Measurement Error

$$
K_k = \frac{e_{EST_{k-1}}}{e_{EST_{k-1}}+e_{MEA_k}}
$$

$$
\begin{array}{l} 
Step1: K_k = \frac{e_{EST_{k-1}}}{e_{EST_{k-1}}+e_{MEA_k}} \\
Step2: \hat{X_k} = \hat{X_{k-1}} + K_k * (Z_k - \hat{X_{k-1}}) \\
Step3:e_{EST_{k}} = (1 - K_k) * e_{EST_{k-1}}
\end{array}
$$

## Data Fusion

$$
\begin{array}{m} 
z_1=30g,\sigma_1=2g \\
z_2=32g,\sigma_2=4g \\
\hat{z} = z_1+k(z_2-z_1) \\
k:kalman Gain,k\in[0,1]  \\
k=0,\hat{z}=z_1;k=1,\hat{z}=z_2 \\
k?->\hat{z}_{min}->\sigma_{\hat{z}_{min}}
\end{array}
$$

$$
\begin{eqnarray}
\sigma_{\hat{z}}^2&=&var(z_1+k(z_2-z_1))=var((1-k)z_1+kz_2) \nonumber    \\
&=&var((1-k)z_1)+var(kz_2) \nonumber    \\
&=&(1-k)^2var(z_1)+k^2var(z_2) \nonumber    \\
&=&(1-k)^2\sigma_1^2+k^2\sigma_2^2
\end{eqnarray}
$$

$$
\begin{array}{m}
\frac{\mathrm{d} \sigma_{\hat{z}}^2}{\mathrm{d} k}=0 \\
-2(1-k)\sigma_1^2+2k\sigma_2^2=0  \\
k=\frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}
\end{array}
$$

## Covariance Matrix

$$
P=\begin{bmatrix}
 \sigma_x^2  & \sigma_{xy}  & \sigma_{xz}\\
 \sigma_{yx} & \sigma_y^2   & \sigma_{yz} \\
 \sigma_{zx} & \sigma_{zy}  & \sigma_z^2
\end{bmatrix}
$$

* transition matrix

$$
\begin{array}{m}
a=\begin{bmatrix}
  x_1 & y_1  & z_1 \\
  x_2 & y_2  & z_2 \\
  x_3 & y_3  & z_3
\end{bmatrix}-\frac{1}{3}
\begin{bmatrix}
  1 & 1  & 1 \\
  1 & 1  & 1 \\
  1 & 1  & 1
\end{bmatrix}
\begin{bmatrix}
  x_1 & y_1  & z_1 \\
  x_2 & y_2  & z_2 \\
  x_3 & y_3  & z_3
\end{bmatrix}
\\
P=\frac{1}{3}\mathbf{a}^\top a
\end{array}
$$

## state space Representation

* Mass-Spring-Damper
	* Elastic coefficient:K
	* Damping coefficient:B
	* mass displacement:X

$$
\begin{array}{m}
m\ddot{x}+B\dot{x}+Kx=F \\
F->u:Input
\end{array}
$$

* state

$$
\begin{eqnarray}
x_1&=&x \\
x_2&=&\dot{x} \\
\dot{x_1} &=& x_2 \\
\dot{x_2} &=& \ddot{x}=\frac{1}{m}u-\frac{B}{m}x_2-\frac{K}{m}x_1
\end{eqnarray}
$$

* measure

$$
\begin{array}{m}
z_1=x=x_1 :positin \\
z_2=\dot{x} = x_2 :velocity
\end{array}
$$

* state space

$$
\begin{eqnarray}
\begin{bmatrix}
  \dot{x_1}  \\
  \dot{x_2} 
\end{bmatrix}&=&
\begin{bmatrix}
   0 & 1 \\
   -\frac{K}{m} & -\frac{B}{m}
\end{bmatrix}
\begin{bmatrix}
  x_1  \\
  x_2 
\end{bmatrix}+
\begin{bmatrix}
  0  \\
  \frac{1}{m} 
\end{bmatrix}u
\\
\dot{X}(t)&=&AX(t)+Bu(t)
\\
\begin{bmatrix}
  z_1  \\
  z_2 
\end{bmatrix}&=&
\begin{bmatrix}
   1 & 0 \\
   0 & 1
\end{bmatrix}
\begin{bmatrix}
  x_1  \\
  x_2 
\end{bmatrix} 
\\
Z(t)&=&HX(t)
\end{eqnarray}
$$

* Discrete
	* $\omega_{k-1}$: Process noise
	* $v_k$: Measurement noise

$$
\begin{eqnarray}
X_k &=& AX_{k-1}+Bu_k+\omega_{k-1} \\
Z_k &=& HX_k+v_k
\end{eqnarray}
$$

## Derivation of Kalman Gain

