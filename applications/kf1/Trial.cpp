#include "Trial.hpp"

// setprecision example
#include <iostream>     // std::cout, std::fixed
#include <iomanip> 
#include <vector>
#include <sstream>
#include <fstream>
//using namespace ::std;

Trial::Trial(int trialNumber, const Parameters& simulatorParameters, const Parameters& estimatorParameters):
  _trialNumber(trialNumber), _simulator(simulatorParameters), _estimator(estimatorParameters)
{
}



void Trial::run()
{
  int i ;
  /*
  std::vector<std::vector<double> > FX;
  std::vector<double> y;
  std::ifstream ready("ytrue.csv");
  std::ifstream readx("xtrue.csv");  
  if(!readx)
        std::cout<<"no xtrue file"<<std::endl;
  else
  {
        
        std::string st;
        while(getline(readx,st))
        {
                 std::istringstream sin(st);
                 std::vector<double> x;
                 std::string st2;
                 while(getline(sin, st2, ','))
                 {
                     x.push_back((double)atof(st2.c_str()));//First one is position, second one is the velocity
                 }
                 FX.push_back(x);
       }
       
       
  } 
   if(!ready)
   {
      std::cout<<"no y file"<<std::endl;
   }
   else
   {
     while(!ready.eof())
     {
         std::string str;
         getline(ready,str);
         double b =(double)atof(str.c_str());
         y.push_back(b);
         //std::cout<<"y is "<<b<<std::endl;
     }
   }
 
  Observation z;
  State realX;
  */
  for(i=0;i<Nsimruns;i++)
  {
  // TODO: Move these out and make them externally controllable
    State x0 = State::Zero();
    StateCovariance P0 = StateCovariance::Identity();//StateCovariance::Zeros()
  
    _simulator.initialize(x0, P0);
    _estimator.initialize(x0, P0);

    _averageNEES = 0;

    //std::cout<<"P0 is \n"<<P0<<std::endl;

    for (int step = 0; step < maxIterations; ++step)
    {
      
      _simulator.step(step);
      Observation z = _simulator.getZ();
      _estimator.step(z,step);
      //z(0,0) = y[step];//fixedPV[0][step];
      if(step<10&i==0)
      {
        //std::cout<<"z is \n"<<z<<std::endl;
      }
      //_estimator.step(z,step);
      
     
      // chi^2 estimate
      State xErr = _simulator.getX()-_estimator.getXEst();//_estimator.getXEst() - _simulator.getX();
      
      //realX(0,0) = FX[step][0];
      //realX(1,0) = FX[step][1]; 
      //State xErr = realX -_estimator.getXEst();

      //double chi2 = xErr.transpose() * _estimator.getPEst().llt().solve(xErr);
      //NISsshist(jj) = innov_kp1'*invPyykp1*innov_kp1;   
      double chi2 = xErr.transpose()*_estimator.getPEst().inverse()*xErr;
      Observation Innov_M = _estimator.getIn().transpose()*_estimator.getPyy().inverse()*_estimator.getIn();
      double Innov = Innov_M(0,0);
    
      //OneNEES.push_back(chi2);
      rowNEES += chi2;
      rowNIS  += Innov;
      //OneNEES[step] = 
      //std::cout << " " << chi2 << std::endl;
      //std::cout<<_estimator.getPEst()<<std::endl;
      if(step<10&&i==0)
      {
        //std::cout<<"simulator X \n"<<_simulator.getX()<<"\n"<<std::endl;//
        //std::cout<<"estimator cov\n"<<_estimator.getPEst()<<"\n"<<std::endl;
        //std::cout<<"xErr is \n"<<xErr<<std::endl;
        //std::cout<<"estimator X is \n"<<_estimator.getXEst()<<std::endl;
        //std::cout<<"\n \n"<<std::endl;
        //std::cout<<"Innov matrix is "<<Innov_M<<std::endl;
      }
     }
     
     AllNEES+= rowNEES;
     AllNIS += rowNIS;
     //std::cout<<"_averageNEES is "<<rowNEES/maxIterations<<std::endl;
 
     rowNEES = 0;
     rowNIS  = 0;
     //AllNEES.push_back(OneNEES);
     
   }
  
  _averageNEES = AllNEES/(maxIterations*Nsimruns);
  _averageNIS  = AllNIS/(maxIterations*Nsimruns);
  
}
