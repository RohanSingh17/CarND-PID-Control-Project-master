# CarND-PID-Control-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[video1]: ./Videos/Only_P.mov "Only P Initial gain"
[video2]: ./Videos/PD.mov "P & D Initial gain"
[video3]: ./Videos/PID.mov "P, I & D Initial gain"
[video4]: ./Videos/PID_TUNING.mov "PID tuned"

The project required us to tune gains of a PID controller which controls steering the
input of the car to keep it driving on the legal portion of the road. The strategy used
was to tune the gains manually and then use twiddle algorithm to optimize gains while
the car is driving on the road. The process is described below:

## Manual Tuning

1. **Proportional gain:-**
    Initially all the gains were set to a value of 0. The proportional gain was increased
    to until the just starts to oscillate about the track. The value of p-gain
    was found to be 0.6. The video also shows the same (_./Videos/Only_P.mov_)
   
2. **Differential gain:-**
    The differential gain was then introduced until the oscillating behaviour of the car
    just disappears. The value of d-gain was found to be 3.0. The video shows the car
    driving after d gain is introduced after intial P-gain (_./Videos/PD.mov_)

3. **Integral gain:-**
   The integral gain was then introduced so that car follows the center of the lane more closely.
   i.e. to eliminate error bias in the controller. The value of I-gain was found to be 
   0.0001. The video stored at the location shows the effect of I-gain (_./Videos/PID.mov_)
   
## Twiddle Tuning

Once the initial values of controller gains were found, the car was able to steer 
around the track. But while driving through the turns car going over the curbs which 
confirmed that parameters need to be optimized while driving through the turns. 

To optimize the PID gains, twiddle algorithm was used as described during lecture videos.
The parameters were tuned until sum of the values of potential changes for PID gains is
less than a tolerance value (0.002). If the parameters are optimized i.e. sum of potential
changes is less than tolerance, the twiddle optimization stops. 

Once a maximum number of iterations are reached, the values of PID gains are reset to original
values so that algorithm is not stuck in local minimum i.e. values need to be optimized
seperately for turns and straight roads.

The video stored at location shows car driving while twiddle algorithm runs in the 
backgroud. (_./Videos/PID_TUNING.mov_)

 