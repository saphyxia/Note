# Kalman Filter

[toc]

## Recursive Processing

$$
\begin{array}{m} 
\hat{x_k} = \hat{x_{k-1}} + K_k * (z_k - \hat{x_{k-1}}) \\
K_k:kalman Gain
\end{array}
$$

* $e_{EST}$: Estimate error 
* $e_{MEA}$: Measurement error

$$
K_k = \frac{e_{EST_{k-1}}}{e_{EST_{k-1}}+e_{MEA_k}}
$$

$$
\begin{array}{l} 
Step1: K_k = \frac{e_{EST_{k-1}}}{e_{EST_{k-1}}+e_{MEA_k}} \\
Step2: \hat{x_k} = \hat{x_{k-1}} + K_k * (z_k - \hat{x_{k-1}}) \\
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
k? \Rightarrow \hat{z}_{min} \Rightarrow \sigma_{\hat{z}_{min}}
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

## State Space Representation

* Mass-Spring-Damper
	* $K$: Elastic coefficient
	* $B$: Damping coefficient
	* $x$: Mass displacement

$$
\begin{array}{m}
m\ddot{x}+B\dot{x}+Kx=F \\
F \Rightarrow u:Input
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
\dot{x}(t)&=&Ax(t)+Bu(t)
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
z(t)&=&Hx(t)
\end{eqnarray}
$$

* Discrete
	* $x_k$: State variables
	* $A$: State matrix
	* $B$: Control matrix
	* $u_k$: Control variables
	* $\omega_{k-1}$: Process noise
	* $z_k$: Measurement variables
	* $H$: Measurement matrix
	* $v_k$: Measurement noise

$$
\begin{eqnarray}
x_k &=& Ax_{k-1}+Bu_{k-1}+\omega_{k-1} \\
z_k &=& Hx_k+v_k
\end{eqnarray}
$$

## Derivation of Kalman Gain

* $\hat{X^-_k}$: Priori estimate

$$
\begin{array}{m}
\hat{x^-_k} = A\hat{x_{k-1}}+Bu_{k-1} \\
z_k = Hx_k \Rightarrow \hat{x_{k_{MEA}}}=H^{-1}z_k
\end{array}
$$

* $\hat{x_k}$: Posterior estimation

$$
\begin{array}{m}
\hat{x_k}=\hat{x^-_k}+G(H^{-1}z_k-\hat{x^-_k}) \\
G\in[0,1] \\
G=0,\hat{x_k}=\hat{x^-_k};G=1,\hat{x_k}=H^{-1}z_k 
\end{array}
$$

$$
\begin{array}{m}
G=K_kH \\
\hat{x_k}=\hat{x^-_k}+K_k(z_k-H\hat{x^-_k}) \\
K_k\in[0,H^{-1}] \\
K_k=0,\hat{x_k}=\hat{x^-_k};K_k=H^{-1},\hat{x_k}=H^{-1}z_k \\
K_k?\Rightarrow \hat{x_k}->x_k
\end{array}
$$

* $e_k$: Error
* $0$: Target
* $P$: Covariance matrix


$$
\begin{array}{m}
VAR(x)=E(x^2)-E^2(x) \\
target=0 \Rightarrow E^2(x)=0 \Rightarrow VAR(x)=E(x^2)
\end{array}
$$

$$
\begin{eqnarray}
e_k&=&x_k-\hat{x_k} \\
P(e_k)& \sim & (0,P) 
\end{eqnarray}
$$

$$
\begin{eqnarray}
P&=&E[ee\top] \\
 &=&E[
\begin{bmatrix}
    e_1 \\
    e_2
\end{bmatrix}
\begin{bmatrix}
    e_1 & e_2
\end{bmatrix}
] \\
&=&
E[
\begin{bmatrix}
    e_1^2 & e_1e_2 \\
    e_2e_1 & e_2^2
\end{bmatrix}
] \\
&=&
\begin{bmatrix}
    \sigma_{e_1}^2 & \sigma_{e_1}\sigma_{e_2} \\
    \sigma_{e_2}\sigma_{e_1} & \sigma_{e_2}^2
\end{bmatrix} 
\end{eqnarray}
$$

$$
\begin{eqnarray}
tr(P)&=&\sigma_{e_1}^2+\sigma_{e_2}^2 \\
tr(P)_{min} & \Rightarrow & \sigma_{min} \Rightarrow \hat{x_k}->x_k \\
K_k? & \Rightarrow & tr(P)_{min}
\end{eqnarray}
$$

$$
\begin{eqnarray}
x_k-\hat{x_k} &=& x_k-(\hat{x^-_k}+K_k(z_k-H\hat{x^-_k})) \\
&=& x_k-\hat{x^-_k}-K_kz_k+K_kH\hat{x^-_k} \\
&=& x_k-\hat{x^-_k}-K_kHx_k-K_kv_k+K_kH\hat{x^-_k} \\
&=& (x_k-\hat{x^-_k})-K_kH(x_k-\hat{x^-_k})-K_kv_k \\
&=& (I-K_kH)(x_k-\hat{x^-_k})-K_kv_k \\
e_k^-&=&x_k-\hat{x^-_k}
\end{eqnarray}
$$

* $(AB)\top=B\top A\top$
* $(A+B)\top=A\top+B\top$

$$
\begin{eqnarray}
P_k &=& E[ee\top] \\
&=& E[(x_k-\hat{x_k})(x_k-\hat{x_k})\top] \\
&=& E[[(I-K_kH)e_k^--k_kv_k][(I-K_kH)e_k^--k_kv_k]\top] \\
&=& E[[(I-K_kH)e_k^--k_kv_k][e_k^-\top(I-K_kH)\top-v_k\top K_k\top]] \\
&=& E[(I-K_kH)e_k^-e_k^-\top(I-K_kH)\top -(I-K_kH)e_k^-v_k\top K_k\top \\
    &-&k_kv_ke_k^-\top(I-K_kH)\top+k_kv_kv_k\top K_k\top]
\end{eqnarray}
$$

* $E(AB)=E(A)E(B)$: A,B Independent

$$
\begin{array}{m}
E[(I-K_kH)e_k^-v_k\top K_k\top]=(I-K_kH)E[e_k^-v_k\top]K_k\top \\
E[e_k^-v_k\top]=E[e_k^-]+E[v_k\top] \\
E[e_k^-]=0,E[v_k\top]=0 \Rightarrow E[(I-K_kH)e_k^-v_k\top K_k\top]=0 \\
also:E[k_kv_ke_k^-\top(I-K_kH)\top]=0
\end{array}
$$

$$
P_k =(I-K_kH)E(e_k^-e_k^-\top)(I-K_kH)\top+K_kE(v_kv_k\top)K_k\top
$$

* $E(e_k^-e_k^-\top)=P_k^-$
* $E(v_kv_k\top)=R$: Measurement nosie Covariance matrix

$$
\begin{eqnarray}
P_k &=& (P_k^--K_kHP_k^-)(I\top-H\top K_k\top)+K_kRK_k\top \\
&=& P_k^- - K_kHP_k^- -P_k^-H\top K_k\top +K_kHP_k^-H\top K_k\top+K_kRK_k\top
\end{eqnarray}
$$

$$
\begin{eqnarray}
(P_k^-H\top K_k\top)\top &=& K_k(P_k^-H\top)\top \\
&=& K_kHP_k^- \\
\Rightarrow tr(K_kHP_k^-)&=&tr(P_k^-H\top K_k\top) \\
\end{eqnarray}
$$

$$
\begin{eqnarray}
tr(P_k)&=&tr(P_k^-)-2tr(K_kHP_k^-) \\
&+&tr(K_kHP_k^-H\top K_k\top)+tr(K_kRK_k\top)
\end{eqnarray}
$$


* $\frac{\mathrm{d} tr(AB)}{\mathrm{d} A}=B\top$


$$
A=\begin{bmatrix}
    a_{11} & a_{12} \\
    a_{21} & a_{22}
\end{bmatrix}
B=\begin{bmatrix}
    b_{11} & b_{12} \\
    b_{21} & b_{22} 
\end{bmatrix}
$$

$$
tr(AB) = a_{11}b_{11}+a_{12}b_{21}+a_{21}b_{12}+a_{22}b_{22} \\
$$

$$
\begin{array}{m}
\frac{\mathrm{d} tr(AB)}{\mathrm{d} A}= 
\begin{bmatrix}
    \frac{\partial tr(AB)}{\partial a_{11}}  & \frac{\partial tr(AB)}{\partial a_{12}} \\
    \frac{\partial tr(AB)}{\partial a_{21}}  & \frac{\partial tr(AB)}{\partial a_{22}} 
\end{bmatrix}=
\begin{bmatrix}
    b_{11} & b_{12} \\
    b_{21} & b_{22} 
\end{bmatrix}= B
\end{array}
$$

* $\frac{\mathrm{d} tr(ABA\top)}{\mathrm{d} A}=2AB$

$$
\begin{array}{m}
\frac{\mathrm{d} tr(P_k)}{\mathrm{d} K_k} = 0 \\
\frac{\mathrm{d} tr(P_k)}{\mathrm{d} K_k} = 0 - 2(HP_k^-)\top +2K_kHP_k^-H\top+2K_kR
\end{array}
$$

* $P_k^-\top=P_k^-$: Covariance matrix

$$
\begin{array}{m}
-P_k^-H\top+K_k(HP_k^-H\top +R)=0 \\
K_k(HP_k^-H\top +R)=P_k^-H\top \\
K_k=\frac{P_k^-H\top}{HP_k^-H\top +R} \\
R\uparrow, K_k->0,\hat{x_k}=\hat{x_k^-} \\
R\downarrow, K_k->H^{-1},\hat{x_k}=H^{-1}z_k
\end{array}
$$

## Priori/Posteriori Error Covariance Matrix

* $P(\omega) \sim N(0,Q)$
* $P(v) \sim N(0,R)$

$$
\begin{eqnarray}
x_k &=& Ax_{k-1}+Bu_{k-1}+\omega_{k-1} \\
z_k &=& Hx_k+v_k
\end{eqnarray}
$$

* Priori estimate

$$
\hat{x^-_k} = A\hat{x_{k-1}}+Bu_{k-1}
$$

* Posteriori estimate

$$
\hat{x_k}=\hat{x^-_k}+K_k(z_k-H\hat{x^-_k})
$$

* Kalman Gain

$$
K_k=\frac{P_k^-H\top}{HP_k^-H\top +R} 
$$

* $P_k^-$?

$$
\begin{array}{m}
P_k^- = E[e_k^-e_k^-\top] \\
e_k^-=x_k-\hat{x_k^-} 
\end{array}
$$

$$
\begin{eqnarray}
e_k^- &=& Ax_{k-1}+Bu_{k-1}+\omega_{k-1}-A\hat{x_{k-1}}-Bu_{k-1} \\
&=& A(x_{k-1}-\hat{x_{k-1}})+\omega_{k-1} \\
&=& Ae_{k-1}+\omega_{k-1}
\end{eqnarray}
$$

$$
\begin{eqnarray}
P_k^- &=& E[(Ae_{k-1}+\omega_{k-1})(Ae_{k-1}+\omega_{k-1})\top] \\
&=& E[(Ae_{k-1}+\omega_{k-1})(e_{k-1}\top A\top +\omega_{k-1}\top)] \\
&=& E[Ae_{k-1}e_{k-1}\top A\top+Ae_{k-1}\omega_{k-1}\top+\omega_{k-1}e_{k-1}\top A\top+\omega_{k-1}\omega_{k-1}\top] \\
&=& E[Ae_{k-1}e_{k-1}\top A\top]+E[Ae_{k-1}\omega_{k-1}\top]+E[\omega_{k-1}e_{k-1}\top A\top]+E[\omega_{k-1}\omega_{k-1}\top]
\end{eqnarray}
$$

* $e_{k-1},\omega_{k-1}$ Independent
* $E[e_{k-1}]=E[\omega_{k-1}]=0$
* $E[Ae_{k-1}\omega_{k-1}\top]=AE[e_{k-1}]E[\omega_{k-1}\top]=0$
* $also:E[\omega_{k-1}e_{k-1}\top A\top]=0$

$$
\begin{eqnarray}
P_k^- &=& AE[e_{k-1}e_{k-1}\top]A\top+E[\omega_{k-1}\omega_{k-1}\top] \\
&=& AP_{k-1}A\top +Q
\end{eqnarray}
$$

* Kalman formula
	* Predict
		* Priori Estimate
		* Priori Error Covariance Matrix
	* Correction
		* Kalman Gain
		* Posteriori Estimate
		* Posteriori Error Covariance Matrix

|Variables|Meaning|
|  ----  | ----  |
|A|state matrix|
|B|control matrix|
|Q|process noise covariance matrix|
|R|measurement noise covariance matrix|
|H|measurement matrix|
|$P_k^-/P_k$|priori/posteriori error covariance matrix|

$$
\hat{x^-_k} = A\hat{x_{k-1}}+Bu_{k-1}
$$

$$
P_k^- = AP_{k-1}A\top +Q
$$

$$
K_k=\frac{P_k^-H\top}{HP_k^-H\top +R}
$$

$$
\hat{x_k}=\hat{x^-_k}+K_k(z_k-H\hat{x^-_k})
$$

$$
\begin{eqnarray}
P_k &=& P_k^- - K_kHP_k^- -P_k^-H\top K_k\top +K_kHP_k^-H\top K_k\top+K_kRK_k\top \\
&=& P_k^- - K_kHP_k^- -P_k^-H\top K_k\top +K_k(HP_k^-H\top+R)K_k\top \\
&=& P_k^- - K_kHP_k^- -P_k^-H\top K_k\top +\frac{P_k^-H\top}{HP_k^-H\top +R}(HP_k^-H\top+R)K_k\top \\
&=& P_k^- - K_kHP_k^- \\
&=& (I-K_kH)P_k^-
\end{eqnarray}
$$

## Extended Kalman Filter

* Nonlinear
    * $P(\omega) \sim N(0,Q)$
    * $P(v) \sim N(0,R)$

$$
\begin{array}{l}
x_k=f(x_{k-1},u_{k-1},\omega_{k-1}) \\
z_k=h(x_k,v_k)
\end{array}
$$

>A normally distributed random variable is no longer normally distributed after passing through a nonlinear system

* Linearzation
	* Taylor Series: $f(x)=f(x_0)+\frac{\partial f}{\partial x}(x-x_0)$
	* Operating Point: 
		* $x_k:\hat{x_{k-1}}$
		* $z_k:\tilde{x_k}$

$$
\begin{array}{l}
x_k=f(\hat{x_{k-1}},u_{k-1},\omega_{k-1})+A(x_k-\hat{x_{k-1}})+W\omega_{k-1} \\
suppose:\omega_{k-1} = 0 \\
f\hat{(x_{k-1}},u_{k-1},0) = \tilde{x_k}\\
A=\frac{\partial f}{\partial x}_{|_{\hat{x_{k-1}},u_{k-1}}} \\
W=\frac{\partial f}{\partial \omega}_{|_{\hat{x_{k-1}},u_{k-1}}}
\end{array}
$$

$$
\begin{array}{l}
eg: \\
x_1=x_1+\sin x_2=f_1 \\
x_2=x_1^2=f_2 \\
A=\frac{\partial f}{\partial x}=
\begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} \\
\frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2}
\end{bmatrix}=
\begin{bmatrix}
1 & \cos x_2 \\
2x_1 & 0
\end{bmatrix}
_{|_{\hat{x_{k-1}},u_{k-1}}} \\
A_k=\begin{bmatrix}
1 & \cos \hat{x_{2_{k-1}}} \\
2\hat{x_{1_{k-1}}} & 0
\end{bmatrix}
\end{array}
$$

$$
\begin{array}{l}
z_k=h(\tilde{x_k},v_k)+H(x_k-\tilde{x_k})+Vv_k \\
suppose:v_k=0 \\
h(\tilde{x_k},0)=\tilde{z_k} \\
H=\frac{\partial h}{\partial x}_{|_\tilde{x_k}} \\
V=\frac{\partial h}{\partial v}_{|_\tilde{x_k}}
\end{array}
$$

* $P(W\omega)\sim N(0,WQW\top)$
* $P(Vv)\sim N(0,VRV\top)$

$$
\begin{array}{l}
x_k=\tilde{x_k}+A(x_k-\hat{x_{k-1}})+W\omega_{k-1} \\
z_k=\tilde{z_k}+H(x_k-\tilde{x_k})+Vv_k
\end{array}
$$

* Extended Kalman Formula

$$
\begin{array}{m}
\hat{x^-_k}=f(\hat{x_{k-1}},u_{k-1},0) \\
P_k^- = AP_{k-1}A\top +WQW\top \\
K_k=\frac{P_k^-H\top}{HP_k^-H\top +VRV\top} \\
\hat{x_k}=\hat{x^-_k}+K_k(z_k-h(\hat{x^-_k},0)) \\
P_k = (I-K_kH)P_k^-
\end{array}
$$