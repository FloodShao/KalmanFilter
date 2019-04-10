# Mathematics
1. Radar measurements:

![](https://github.com/FloodShao/KalmanFilter/raw/master/fig/RadarMeasurement.png)

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

2. The selection of process covariance Q:

There are two covariance matrix we need to estimate, one is **Process covariance matrix Q** the other is  **Measurement covariance R**. Generally, R can be obtained from the sensor calibration procedure. However, Q is not easy to obtained, because Q is often related to situations such as wheel slip. 

In general, Q can be calculated as (Q is symmetrical)
$$
	Q = G * G^T * \sigma^2
$$
where in a CV model, $G = [0.5dt^2, 0.5dt^2, dt, dt]$ and $\sigma^2$ indicates the influenced acceleration. In paper "Empirical Evaluation of Vehicular Models for Ego Motion Estimation " (2011), the typical process noise is shown below:
![](https://github.com/FloodShao/KalmanFilter/raw/master/fig/Typical%20Process%20Noise.png)

Typically, the acceleration process noise is $8.8m/s^2$. As such, in the original start code of Udacity sets $ax = ay = 9.0$.

However, the final process covariance matrix is as follows:
$$
Q = \left[
\begin{matrix}
0.25dt^4 & 0. & 0.5dt^3 & 0. \\
0. & 0.25dt^4 & 0. & 0.5dt^3 \\
0.5dt^3 & 0. & dt^2 & 0. \\
0. & 0.5dt^3 & 0. & dt^2 \\
\end{matrix}
\right]
$$
which implies the independency between x direction and y direction. 

I have tried the correlated version, however, the results are bad. Therefore, it is better to assume the independency of x direction and y direction.


3. The selection of Measurement Covariance R:

In original code, the R matrix of radar measurement is
$$
\left[
\begin{matrix}
0.9 & 0. & 0. \\
0. & 0.009 & 0. \\
0. & 0. & 0.9 \\
\end{matrix}
\right]
$$
However, the radar can get an accurate measurement on the distance and velocity however, the orientation measurement may not be accurate enough. (An accurate orientation estimation requires a narrow radiation beam, which is quite hard especially for middle range radar)
So I believe the correct R matrix of radar measurement should be :
$$
\left[
\begin{matrix}
0.009 & 0. & 0. \\
0. & 0.09 & 0. \\
0. & 0. & 0.009 \\
\end{matrix}
\right]
$$

Besides, Lidar measurements is more accurate than the radar measurement. So I change the Lidar measurement covariance to :
$$
\left[
\begin{matrix}
0.0025 & 0. \\
0. & 0.0025  \\
\end{matrix}
\right]
$$


