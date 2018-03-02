1:please don't change deltaT for now because the G matrix(check matlab code) will change along with delta T, in the code I just write G = [0.005,0.1]' for deltaT = 0.1

2:in prediction part, I add the input G*u[];

3:initilization covariance(P0) be Identity instead of Zeros 

4:In file kp1, I just add excutable file for testTrail.cpp to test if it is the same as Matlab code

5:Kalman fiter part, original code for _PEst is 
  _PEst = X * _PPred* X.transpose() + K * _observationModel.getR() * K.transpose();
  I change it to be the same as Matlab code ("Pkp1_plus" is _PEst in C++) 
  _PEst = X * _PPred;
  
6:In Matlab code it does average for a whole NEESsamps matrix(10 by 201)

7:No NIS yet, add later
