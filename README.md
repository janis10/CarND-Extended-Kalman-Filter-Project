# Extended Kalman Filter Project
**Disclaimer:** The code in this repository is derived from the [Extended Kalman Filter project of Udacity](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). If you like it feel free to support it in the above link! 

### Description
We implement an Extended Kalman Filter (EKF) to estimate the state of a moving car given noisy LiDAR and RADAR measurements. The equations of motions are assumed as follows:
```
px[k+1] = px[k+1] + vx[k] * dt + wpx[k]
py[k+1] = py[k+1] + vy[k] * dt + wpy[k]
vx[k+1] = vx[k] + wvx[k]
vy[k+1] = vy[k] + wvy[k]
```
i.e., a linear system, where `px` and `py` denote the position of the car along the x and y axes, and `vx` and `vy` the corresponding velocities. The quantity `w` is assumed to be Gaussian white noise. 
The fusion of the two types of measurements is performed as follows:
* when the incoming measurement is from the LiDAR, its output is `y[k] = (px[k], py[k])`, which is a linear function of the state and, hence, we use a regular Kalman Filter prediction and update step. 
* when the incoming measurement is from the RADAR, its output is `y[k] = (sqrt(px[k]^2+py[k]^2), atan(py[k]/px[k]), (px[k]vx[k]+py[k]vy[y])/sqrt(px[k]^2+py[k]^2))`, which is a non-linear function of the state and, hence, we use an Extended Kalman Filter prediction and update step. 

### Dependencies
1. Udacity's [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases). 
2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), which can be installed using the provided bash scripts by Udacity for either Linux or Mac systems. 
3. [Eigen](https://eigen.tuxfamily.org) template library for linear algebra.
4. [cmake](https://cmake.org/install/) >= 3.5
5. make >= 4.1 (Linux, Mac)
6. gcc/g++ >= 5.4

### Build and run
Once the dependencies are installed, the main program is built and run as follows:
```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```
If successful the output should be:
```
Listening to port 4567
Connected!!!
```
Then, one just opens the simulator and executes a test run. 

For more details on how main.cpp uses uWebSocketIO to communicate with the simulator is as follows see the [original Udacity repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

<!-- The protocol that main.cpp uses for uWebSocketIO in communicating with the simulator is as follows:

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

--- -->