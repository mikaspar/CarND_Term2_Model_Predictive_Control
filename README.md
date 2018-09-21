# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Description

The model predicitve controller was implemented for the vehicle motion control in order to keep the vehicle on the desired trajectory.

The model is based on kinematic bicycle model. 
The state vector include:

                           position x
                           position y
                           velocity
                           cross track error
                           heading error

The cost function include following parameters and functions:
                           
                           heading error multiplied by factor f_e
                           position error multiplied by factor f_xy
                           speed error 
                           steering value multiplied by factor f_d
                           throttle value
                           steering gradient multiplied by factor f_ddiff
                           throttle gradient
                           
                              
The mapped waypoints of the trajectory are transformed to the vehicle coordinate system before passing to the MPC.

The model deals with the latency of 100ms. The latency is considered in the model by adding the predicted state change in the time of the latency to the prediction.

Manual parameter tuning was done in order to find values with the maximal performace in terms of driving safe on track with the maximal possible speed with shortest possible turnover time. 

Resulting values 7 steps with 0.1 step duration.

Very high parameter of steering factor gradient was used in order to "optimize" the curve driving. This could be also done by optimizing the trajectory itself. (driving thru the optimal curve radius) 







                           
