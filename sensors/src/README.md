# Kalman Filter

Sensor fusion algorithm using LiDAR and RADAR data to track moving objects, predicting and updating dynamic state estimation.



This project implements the  Kalman Filter for tracking a moving object. The intention is to measure the object's position and velocity.

Since we are only interested in 2D movement, the state variables are `px`,`py`,`vx`,`vy`. The sensors used for detecting the object are RADAR and Laser (LiDAR). The advantage of having multiple types of sensors which are fused is higher performance. The laser has better position accuracy, but the RADAR measures the velocity directly by using the Doppler Effect.
 
This project is implemented in C++ using the Eigen library. The source code is located in `FusionEKF.cpp` and `kalman_filter.cpp` files in the `src` folder above. 




## Prediction
The Kalman Filter works in a sequence of `Prediction` - `Update` cyclically computed. Each time a new measurement is received from one of the sensors, a prediction is computed first. 

The Kalman Filter prediction equations are the following:

![prediction](images/prediction.JPG)

where `x` is the current state space, `x'` is the predicted state space and `u` is the control, which is zero in this case since we don't control the object's motion.

`F` is the transition matrix describing the object's dynamics, `P` is the covariance matrix representing the uncertainty in the state space values, and `Q` is the prediction noise.

Object position and velocity prediction is done assuming linear motion with zero acceleration,

![linear_motion](images/linear_motion.JPG)


which written in a matrix form is:

![matrix_motion](images/matrix_motion.JPG)


To be able to predict state estimation, elapsed time and transition matrix are needed. The `dt` is simply the time between the previous timestamp and the one carrying the new measurement.

```
// dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
```

`F_` is a matrix that, when multiplied by `x`, predicts where the object will be after time `dt`.
Given that our state space `x` is `(px, py, vx, vy)` the `F_` matrix for an object moving with constant velocity is:

```
// the transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
```

There is only one prediction function, regardless if it's calculated for a RADAR or LiDAR measurement. The same equations are used independently from the sensor type. 

```
//state prediction
x_ = F_ * x_;
MatrixXd Ft = F_.transpose();
P_ = F_ * P_ * Ft + Q_;
```

`P_` is the covariance matrix to which the process noise is added. When initialized, the `P_` matrix gets the values set below. 

```
// state covariance matrix P
ekf_.P_ = MatrixXd(4, 4);
ekf_.P_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1000, 0,
           0, 0, 0, 1000;
```

Things to notice here are that the uncertainty of `px` and `py` is small, only one meter. The velocities `vx` and `vy` have a high, 1000 meters per second, uncertainty. These values get lower every cycle a measurement update is performed.

The `Q_` process noise takes into account that the prediction is known not to be accurate and is meant to increase the uncertainty after each prediction. 

Since the acceleration is unknown we can add it to the noise component. So, we have a
random acceleration vector `ν` in this form:

![acceleration_noise](images/acceleration_noise.JPG)


The acceleration is a random `[ax, ay]` vector with zero mean and standard deviation `σax` and `σay`.

`ax` and `ay` are assumed uncorrelated noise processes, so after combining everything in one matrix we obtain our 4 by 4 `Q_` matrix:

![Qmatrix](images/Qmatrix.JPG)


The `σax` and `σay` implemented as `noise_ax` and `noise_ay` are given for the project to be `9.0` meters per second squared.

```
// the process noise covariance matrix Q_
  float noise_ax = 9.0f;
  float noise_ay = 9.0f;
  float dt4 = (dt*dt*dt*dt)/4;
  float dt3 = (dt*dt*dt)/2;
  float dt2 = (dt*dt);
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
             0, dt4*noise_ay, 0, dt3*noise_ay,
             dt3*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3*noise_ay, 0, dt2*noise_ay;
```


## Measurement Update - LiDAR

For the *Update* step, we need to know which sensor is sending the data. This information is relevant to determine the measurement update equations.

The Kalman Filter measurement update equations are the following:

![measurement_update](images/measurement_update.JPG)

where `z` is the measurement vector and `x'` is the predicted state space. `K` is the Kalman gain just used for calculation purposes and `R` is the measurement noise.

For a lidar sensor, the `z` vector contains the `[px,py]` measurements. `H` is the matrix that projects your belief about the object's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position.

The state vector `x` contains information about [p_x, p_y, v_x, v_y] whereas the `z` vector will only contain [px, py]. Multiplying `H` allows us to compare `x'`, the predicted state, with `z`, the sensor measurement.

```
 // measurement matrix - laser
H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
```

The implementation for the measurement update is given below. `x_` is the predicted state and `z_pred` is `x_` in the form of the measurement vector. Subtracting the two gives us the error `y`.

```
VectorXd z_pred = H_ * x_;
VectorXd y = z - z_pred;
MatrixXd Ht = H_.transpose();
MatrixXd S = H_ * P_ * Ht + R_;
MatrixXd Si = S.inverse();
MatrixXd PHt = P_ * Ht;
MatrixXd K = PHt * Si;

//new estimate
x_ = x_ + (K * y);
long x_size = x_.size();
MatrixXd I = MatrixXd::Identity(x_size, x_size);
P_ = (I - K * H_) * P_;
```
 
`R_` is the measurement noise that is taken into account knowing the precision of each sensor. For laser `R_` is:

```
//measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
```

This is LiDAR's best feature, being able to detect objects with a precision of about 2 centimeters.



