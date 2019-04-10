# Mathematics
1. Radar measurements:

![avatar](https://github.com/FloodShao/KalmanFilter/raw/master/fig/Radar%20Measurement.png)

As the graph shows, the radar can only measure the radial distance $\rho$ and radial velocity $\dot \rho$. As such the measurement transformation function from state (px, py, vx, py) to the radar measurement ($\rho$, $\phi$, $\dot \rho$) can be written as:

$$
\rho = \sqrt{px^2 + py^2} \\
\phi = atan \frac{py}{px} \\
\dot \phi = vx * cos \phi + vy * sin \phi = \frac{vx * px + vy * py}{\rho}
$$

and the Jacobian Matrix of Hj can be written as :
$$
\left[
\begin{matrix}
px/\rho & py/rho & 0 & 0 \\
-py/\rho^2 & px/\rho^2 & 0 & 0\\
\frac{py(py*vx - px*vy)}{\rho^2} & \frac{px(px*vy - py*vx)}{\rho^2} & px/\rho & py /\rho \\
\end{matrix}
\right]
$$
given that 
$$
d(tan x) = \frac{1}{1+x^2}
$$


