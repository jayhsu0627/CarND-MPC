# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Project Instructions and Rubric

### The Model
The MPC is an advanced control technique for complex control problems. MPC is an optmization problem to find the best set of control inputs that minimizes the cost functions based on the prediction (dynamical) model. The MPC controller consists of:

* **Prediction Horizon**

The Prediction Horizom is the duration over which future predictions are made. It comprehend the number of timesteps N in the horizon and the time elapse of each timestep dt.

The number N also determines the number of variables optmized by the controller. So, higher N will result in extra computational cost.



* **State**

The state consists of sytem variables and errors references: [x,y,psi,v,cte,epsi]. x and y stand for the vehicle position, psi the vehicle orientation, v the vehicle speed and finally, cte and epsi stand for the cross track error and orientation error of the vehicle related to the reference.


* **Model (Update equations)**

The followind equations updates the prediction model at every timestep:

equation


Lf measures the distance between the front of the vehicle and its center of gravity. f(x) is the evaluation of the polynomial f at point x and psidest is the tangencial angle of the polynomial f evaluated at x.


* **Timestep Length and Elapsed Duration (N & dt)**

For this project, we followed an empirical approach of trial and error to choose the horizom values. We tried for N values between 10 and 20 and for dt 0.05 and 0.1. The best result was achieved with N=10 and dt=0.1, givin a time horizom of 1 second. 

* **Polynomial Fitting and MPC Preprocessing**

Before fitting the path returned from the simulator, we have to preprocess in order to move the points to the origin (x=0, y=0) and also rotate the path to follow the car orientation.
~~~~
for(unsigned int i=0; i < ptsx.size(); i++){
	//shift points to the initial point
	double shift_x = ptsx[i] -px;
	double shift_y = ptsy[i] -py;

	//rotate 90 degrees
	//http://planning.cs.uiuc.edu/node99.html
	ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
	ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0-psi));
}
~~~~
* **Latency**

In order to deal with the latency, we have to predict the next state before calling the MPC solver. It can be acoomplished using the Model equations. A latency of 100ms is defined.

```
double dt = 0.1
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * (dt ));
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * (dt ));
fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * (dt ) );
fg[1 + v_start + t] = v1 - (v0 + a0 * (dt ));
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * (dt )));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * (dt ));

```