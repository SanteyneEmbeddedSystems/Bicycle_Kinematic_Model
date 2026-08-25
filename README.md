# Bicycle kinematic model

This repo presents and demonstrates the bicycle kinematic model.

The bicycle kinematic model is a simplified mathematical model used to describe
and simulate the motion of a wheeled vehicle, such as a car, in two dimensions.
This model is commonly used in robotics and autonomous vehicle navigation
because it captures the key aspects of how vehicles steer and move without
involving complex dynamics. By representing the vehicle as a two-wheeled
"bicycle", the model reduces the front and rear wheels to two points connected
by a rigid body.

The vehicle is assumed to move on a planar surface. The vehicle body is rigid,
and the wheels are assumed to roll without lateral slip.

## Parameters

![Bicycle model parameters](doc/bicycle_model.svg)

The ground is associated with the frame $R_0 (O_0, \overrightarrow{x_0},
\overrightarrow{y_0}, \overrightarrow{z_0} )$ where $\overrightarrow{z_0}$
points vertically upward.

The vehicle is associated with the frame $R_1 (O_1, \overrightarrow{x_1},
\overrightarrow{y_1}, \overrightarrow{z_1} )$  where:
* $O_1$ lies on the longitudinal axis of the vehicle.
* $\overrightarrow{x_1}$ points to the right of the vehicle.
* $\overrightarrow{y_1}$ points to the front of the vehicle.
* $\overrightarrow{z_1} = \overrightarrow{z_0} = \overrightarrow{z}$

Let $F$ be the point at the center of the front wheel.  
Let $R$ be the point at the center of the rear wheel.  
Let $L$ be the wheelbase of the vehicle, i.e. the distance $\overline{RF}$.  
We have $\overline{RO_1} + \overline{O_1F} = \overline{RF} = L$.

The steering angle of the front wheel is $\delta_F$.  
The steering angle of the rear wheel is $\delta_R$.

The body slip angle is $\varphi$.

The position of the vehicle in the $R_0$ frame is described by the parameters
$x$, $y$ and $\theta$ such that:
* $\overrightarrow{O_0O_1} = x \times \overrightarrow{x_0} + y \times
\overrightarrow{y_0}$
* $\theta = \left(\overrightarrow{x_0}, \overrightarrow{x_1} \right) $

The aim of this study is to give the time derivatives of $x$, $y$ and $\theta$
as function of the speed of the vehicle ($V_{O_1}$), the steering angles
($\delta_F$ and $\delta_R$) and the geometry of the vehicle.

## Model computation

### Yaw rate

According to the rigid-body kinematics formula, we have:
```math
\overrightarrow{V_{F\in R_1/R_0}} = \overrightarrow{V_{R\in R_1/R_0}}
+ \overrightarrow{FR} \wedge \overrightarrow{\Omega_{R_1/R_0}}
```

This can be written as:
```math
V_F \overrightarrow{x_F} = V_R \overrightarrow{x_R}
- L\overrightarrow{y_1} \wedge \dot \theta \overrightarrow{z}
```

Where:
* $\overrightarrow{x_F} =
-\sin(\delta_F)\overrightarrow{x_1}+\cos(\delta_F)\overrightarrow{y_1}$
* $\overrightarrow{x_R} =
-\sin(\delta_R)\overrightarrow{x_1}+\cos(\delta_R)\overrightarrow{y_1}$

In the $R_1$ frame:
```math
V_F \left( -\sin(\delta_F)\overrightarrow{x_1} 
   + \cos(\delta_F)\overrightarrow{y_1} \right)
= V_R \left( -\sin(\delta_R)\overrightarrow{x_1}
   + \cos(\delta_R)\overrightarrow{y_1} \right)
   - L \dot \theta \overrightarrow{x_1}
```

Projecting onto $\overrightarrow{x_1}$ gives:
```math
\begin{aligned}
&-V_F \sin(\delta_F) = -V_R \sin(\delta_R) - L \dot \theta \\
\iff &\dot \theta = \frac{1}{L} \times
\left( V_F \sin(\delta_F) -  V_R \sin(\delta_R) \right)
\end{aligned}
```

Projecting onto $\overrightarrow{y_1}$ gives:
```math
\begin{aligned}
&V_F \cos(\delta_F) = V_R \cos(\delta_R) \\
\iff &V_F = V_R \frac{\cos(\delta_R)}{\cos(\delta_F)}
\end{aligned}
```

Therefore:
```math
\begin{aligned}
\dot \theta
&= \frac{1}{L} \times \left(
V_R \frac{ \cos(\delta_R)}{\cos(\delta_F)} \sin(\delta_F) -  V_R \sin(\delta_R)
\right) \\
&= V_R\frac{\cos(\delta_R)}{L}\times\left(\tan(\delta_F)-\tan(\delta_R)\right)
\end{aligned}
```

### Body slip angle

The formula obtained above gives the yaw rate as a function of the speed at the
rear axle.  
We would like to express it instead as a function of the speed at point $O_1$.  
Therefore, we need to express the speed at point $R$ as a function of the speed
at point $O_1$.

According to the rigid-body kinematics formula, we have:
```math
\overrightarrow{V_{R\in R_1/R_0}} = \overrightarrow{V_{O_1\in R_1/R_0}}
+ \overrightarrow{RO_1} \wedge \overrightarrow{\Omega_{R_1/R_0}}
```

Therefore:
```math
V_R = V_{O_1}\frac{\cos(\varphi)}{\cos(\delta_R)}
```

We now need to express $\varphi$ as a function of the input parameters
$\delta_R$ and $\delta_F$ as well as the vehicle dimensions ($\overline{O_1F}$
and $\overline{O_1R}$ ).

![Body slip angle](doc/body_slip_angle.png)

* $\alpha = \delta_F - \varphi $
* $\beta = \delta_F - \delta_R $

Considering triangle $O_1FI$, one can write:
```math
\frac{\overline{O_1F}}{\sin(\alpha)} =
\frac{\overline{IF}}{\sin \left( \frac{\pi}{2} + \varphi \right)}
```

Considering triangle $RFI$, one can write:
```math
\frac{\overline{RF}}{\sin(\beta)} =
\frac{\overline{IF}}{\sin \left( \frac{\pi}{2} + \delta_R \right)}
```

Therefore:
```math
\begin{aligned}
\frac{1}{\overline{IF}}
=&\frac{1}{\overline{O_1F}}
\frac{\sin(\alpha)}{\sin\left(\frac{\pi}{2}+\varphi\right)}
=
\frac{1}{\overline{RF}}
\frac{\sin(\beta)}{\sin \left( \frac{\pi}{2}+\delta_R \right)} \\

\iff &\frac{1}{\overline{O_1F}}  \frac{\sin(\delta_F - \varphi)}{\cos(\varphi)}
=
\frac{1}{\overline{RF}} \frac{\sin(\delta_F - \delta_R)}{\cos(\delta_R)} \\

\iff &\frac{1}{\overline{O_1F}} \frac{\sin(\delta_F) \cos(\varphi)
    - \cos(\delta_F)\sin(\varphi)}{\cos(\varphi)}
=
\frac{1}{\overline{RF}}\frac{\sin(\delta_F)\cos(\delta_R)
    - \cos(\delta_F)\sin(\delta_R)}{\cos(\delta_R)} \\

\iff &\frac{1}{\overline{O_1F}}\left( \sin(\delta_F)
    - \cos(\delta_F)\tan(\varphi) \right)
=
\frac{1}{\overline{RF}}\left( \sin(\delta_F)
    - \cos(\delta_F)\tan(\delta_R)\right) \\

\iff &\frac{1}{\overline{O_1F}}\left( \tan(\delta_F) - \tan(\varphi) \right)
=
\frac{1}{\overline{RF}}\left( \tan(\delta_F) - \tan(\delta_R)\right) \\

\iff& \tan(\varphi) =
\tan(\delta_F) - \frac{\overline{O_1F}}{\overline{RF}} \times
\left( \tan(\delta_F) - \tan(\delta_R) \right) \\

\iff& \tan(\varphi) =
\left( 1 - \frac{\overline{O_1F}}{\overline{RF}} \right) \tan(\delta_F) +
\frac{\overline{O_1F}}{\overline{RF}} \tan(\delta_R) \\

\iff& \tan(\varphi) =
\frac{\overline{RO_1}}{\overline{RF}}\tan(\delta_F) +
\frac{\overline{O_1F}}{\overline{RF}} \tan(\delta_R) \\

\iff& \varphi = \arctan \left(
\frac{\overline{RO_1}}{\overline{RF}}\tan(\delta_F) +
\frac{\overline{O_1F}}{\overline{RF}} \tan(\delta_R) \right)\\

\end{aligned}
```

### Speed at point $O_1$

We now want to determine the speed at point $O_1$:
```math
\overrightarrow{V_{{O_1}\in R_1/R_0}} = \begin{pmatrix}
\dot x\\ 
\dot y
\end{pmatrix}_{R_0}
```

We also have:
```math
\overrightarrow{V_{{O_1}\in R_1/R_0}} = V_{O_1}\overrightarrow{u_1}
```

Knowing that $\overrightarrow{u_1}=-\sin(\varphi+\theta)\overrightarrow{x_0}
+\cos(\varphi+\theta)\overrightarrow{y_0} $

We obtain:
```math
\overrightarrow{V_{{O_1}\in R_1/R_0}} =
V_{O_1} \left( -\sin(\varphi+\theta)\overrightarrow{x_0}
+\cos(\varphi+\theta)\overrightarrow{y_0} \right)
```

Therefore:
```math
\left\{
\begin{aligned}
\dot x &= V_{O_1} \times -\sin( \varphi + \theta ) \\
\dot y &= V_{O_1} \times \cos( \varphi + \theta )
\end{aligned}
\right.
```

### Bicycle model equations

To summarize, the kinematic bicycle model is:
```math
\left\{
\begin{aligned}
\dot x &= -V_{O_1}\sin( \theta + \varphi ) \\
\dot y &= V_{O_1}\cos( \theta + \varphi ) \\
\dot \theta &= V_{O_1}\frac{\cos(\varphi)}{L}\times\left(\tan(\delta_F)
  -\tan(\delta_R)\right) \\
\varphi &= \arctan \left(\frac{\overline{R{O_1}}}{L}\tan(\delta_F)
  +\frac{\overline{{O_1}F}}{L} \tan(\delta_R) \right)
\end{aligned}
\right.
```
