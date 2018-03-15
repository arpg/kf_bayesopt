//#include "opencv2/core/core.hpp"
#include "Trial.hpp"

#include <sysexits.h>
#include <iostream>
#include <fstream>
#include <string>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
//using namespace cv;
int main(int argc, const char* argv[])
{
  double pn = 0;
  double on = 0;
  vector<double> n; 
  if(argc!=1)
  {
    cout<<"just one input"<<endl;
  }

  ifstream read("/home/zhaozhong/ekf_bayesopt_noupload/applications/kf1/data/pn_on.csv");  //modify the location!!!
  if(!read)
  {
    cout<<"cannot open the file"<<endl;
  }
  else
  {
    string st;
    while(getline(read, st, ','))
       {
         n.push_back((double)atof(st.c_str()));
       }
    //cout<<"n0 is "<<n[0]<<","<<"n[1] is "<< n[1]<<endl;
  }
  read.close();

  Parameters correctParameters(1, 1, 0.1);
  Parameters wrongParameters(n[0], n[1], 0.1);//pn, on

  //  Trial t(1, correctParameters, wrongParameters);
  Trial t(1, correctParameters, wrongParameters);

  t.run();

  double J_NEES = sqrt((log(t.getAverageNEES()/2))*(log(t.getAverageNEES()/2)));
  
  //std::cout << "Average NIS=" << J_NIS << std::endl;


  ofstream output("/home/zhaozhong/ekf_bayesopt_noupload/applications/kf1/data/NEES.csv",ios::out);//location needs to be modified
  output<<J_NEES;
  output.close();

  //ofstream his("/home/zhaozhong/ekf_bayesopt_noupload/applications/kf1/his/His.csv",ios::out|ios::app);
  //his<<NEES<<","<<n[0]<<","<<n[1]<<endl;
  //his<<NEES<<","<<n[0]<<endl;
  //his.close()
  return EX_OK;
}
