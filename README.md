# State space reduction
Reduce dimensions of a state space system while preserving model output accuracy, by balanced realization.

This example uses a 3 mass spring damper system. 

![3mass](https://user-images.githubusercontent.com/37086122/56441950-c4677900-62bc-11e9-992d-7cdd5a8c9b83.PNG)

The original system has 6 states: speed and position of each mass. Reduced system matrices are stores as Ared, Bred, and Cred. Different amounts of reduction were attempted and results are plotted. 

![p2](https://user-images.githubusercontent.com/37086122/56442137-8d459780-62bd-11e9-8ed3-979abf327f8c.jpg)

We can see that the system behavior deviates more after 3 states are discarded. This is confirmed by checking diagonal values of T * wc * T', where T is the transformation to realize the new system, and wc is the original system's controllability gramian. 

![image](https://user-images.githubusercontent.com/37086122/56442301-35f3f700-62be-11e9-84d4-f3656356bfbd.png)

These diagonal values can be used to predict output accuracy. The 3rd element is much smaller than the 1st and 2nd, meaning removing 3 states will significantly affect the quality of your reduced model. Keep in mind that regardless of how many states you take away, states of the reduced model do not correspond to states of your physical system anymore. It's the output Y that is still accurate.

## Purpose
Once you have your reduced system, it can be used in model based algorithms such as Kalman filters and model predictive control. The advantage here is since it's a smaller state space, your code will run faster.
