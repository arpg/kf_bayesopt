#include <iostream>
#include <iomanip>
#include <string>

#include <Eigen/Eigen>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
//#include <sophus/sophus.hpp>


int main(int argc, char* argv[]) {
  Eigen::Vector2d m0( 0, 0 );
  Eigen::Matrix2d p0;
  p0 << 2, 0, 2, 0 ;

  // Model A: simple NCV kinematics, no velocity drag terms
  Eigen::Matrix2d a;
  a <<  0, 1, 0, 0;
  Eigen::Vector2d b( 0, 1 );
  Eigen::Vector2d gam( 0, 1 );
  Eigen::Vector2d c( 1, 0 );
  double wt = 1; 					// *ACTUAL* AWG process noise intensity: (m/s^2)^2
  double vt = 0.005; 			// %AWG measurement noise intensity: m^2


  /*
 * // Model B: NCV kinematics with velocity drag term
double bd = -2;
Eigen::Matrix a( 0, 1, 0, bd );
Eigen::Matrix b( 0, 1 );
Eigen::Matrix gamma( 0, 1 );
Eigen::Matrix c( 1, 0 );
double wt = 0.05; 			// *ACTUAL* AWG process noise intensity: (m/s^2)^2
double vt = 0.005; 			// AWG measurement noise intensity: m^2
*/

  // The state-space model (originally was built using a MATLAB builtin `ss')
  // is of the form:
  // x' = a x + b u
  // y  = c x + d u

  double deltaT = 0.1;
  Eigen::Matrix2d f;
  f << 1, deltaT, 0, 1 ;
  Eigen::Vector2d g( 0.5*deltaT*deltaT, deltaT );
  Eigen::Vector2d h = c;

  // Find the measurement noise covariance
  double rt = vt / deltaT;


  // Use Van Loan method to find covariance of process noise
  // This will segfault, for sure.
  Eigen::Matrix2d m;
  m.row(0) << -a, gam*wt*gam.transpose();
  m.block(1,0,2,2) << Eigen::Matrix2d::Zero(2);
  m.col(2).tail(2) << a.transpose();
  m *= deltaT;
  Eigen::Matrix2d m_exp(m.exp());
  Eigen::Matrix2d invFQblock;
  invFQblock << m_exp(0,0), m_exp(1,0), m_exp(0,1), m_exp(1,1) ;
  Eigen::Matrix2d qt = f*invFQblock;

  // Open-loop Control input
  Eigen::Vector2d tv(Eigen::Vector2d::LinSpaced(deltaT, 0, 12));
  tv *= 0.75;
  Eigen::Vector2d u = tv.cos(); 	// acceleration input
  tv /= 0.75;


  // Run chi-square tests on simulated filtering runs:
  // Specify KF model of dynamics:
  // *----* MAY BE MISMATCHED TO ACTUAL SYSTEM MODEL !!!
  Eigen::Matrix2d fkf;
  fkf <<  1, deltaT, 0, 1 ;
  Eigen::Vector2d gkf( 0.5*deltaT*deltaT, deltaT );
  Eigen::Vector2d hkf( 1, 0 );

  ///// Specify assumed DT noise covariances for KF:
  // MAY BE MISMATCHED TO ACTUAL VALUES !!! ***
  Eigen::Matrix2d qkf;
  qkf << 5e-3, 0, 0, 1e-3 ; // diag([0.5 1])% Qtrue;%  %%treat this as the main tuning knob
  double rkf = rt; // assume we can get this from sensor specs

  size_t max_iterations = 50;
  Eigen::Matrix2d NEESsamps = Eigen::Matrix2d::Zero(max_iterations,tv.cols());
  Eigen::Matrix2d NISsamps = Eigen::Matrix2d::Zero(max_iterations,tv.cols());

  for( size_t ii = 0; ii < max_iterations; ii++ ) {

  }


  /*
  // Begin Nisar's plot generation.
  for ss=1:Nsimruns
      %%%1. %%Generate true trajectory and measurements from system

      xk_truehist = zeros(2,length(tvec));
      ykhist = zeros(1,length(tvec));
      xk = mvnrnd(m0,P0); %sample initial robot state
      for jj=1:length(tvec)

          wk = mvnrnd(zeros(1,2),Qtrue);
          xkp1 = F*xk' + G*u(jj) + wk';
          vkp1 = mvnrnd(zeros(1,1),Rtrue)';
          ykp1 = H*xkp1 + vkp1;

          xk_truehist(:,jj) = xkp1;
          ykhist(:,jj) = ykp1;
          xk = xkp1';
      end

      %%% 2. Kalman Filter equations with simple NCV model

      %%Run the Kalman filter updates
      mk = m0;
      Pk = P0;
      mk_filt_hist = zeros(2,length(tvec));
      Pk_filt_hist = zeros(2,2,length(tvec));
      innovk_hist = zeros(1,length(tvec));
      Pyyk_hist = zeros(1,1,length(tvec)); %%store measurement innovation covar
      NEESsshist = zeros(1,length(tvec));
      NISsshist = zeros(1,length(tvec));
      for jj=1:length(tvec)

          %%Perform prediction step
          mkp1_minus = F*mk' + G*u(jj);
          Pkp1_minus = F*Pk*F' + Qkf;

          %%Compute Kalman gain
          Pyykp1 = H*Pkp1_minus*H' + Rtrue;
          Pyykp1 = 0.5*(Pyykp1 + Pyykp1');
          Kkp1 = Pkp1_minus*H'/(H*Pkp1_minus*H' + Rtrue);
          %%Perform measurement update step
          ykp1_report = ykhist(:,jj); %simulate the reporting of data from sensor
          ykp1_pred = H*mkp1_minus; %predicted measurement
          innov_kp1 = ykp1_report - ykp1_pred; %compute meas innovation
          mkp1_plus = mkp1_minus + Kkp1*innov_kp1; %compute update to state mean
          Pkp1_plus = (eye(2) - Kkp1*H)*Pkp1_minus; %compute update to covar

          mk = mkp1_plus';
          mk_filt_hist(:,jj) = mkp1_plus;
          Pk = Pkp1_plus;
          Pk_filt_hist(:,:,jj)= Pkp1_plus;
          innovk_hist(:,jj) = innov_kp1;

          %%Compute and store NEES and NIS statistics:
          invPkp1 = inv(Pkp1_plus);
          invPyykp1 = inv(Pyykp1);
          NEESsshist(jj) = ...
              (xk_truehist(:,jj) - mkp1_plus)'*invPkp1*(xk_truehist(:,jj) - mkp1_plus);
          NISsshist(jj) = innov_kp1'*invPyykp1*innov_kp1;
      end
      NEESsamps(ss,:) = NEESsshist;
      NISsamps(ss,:) = NISsshist;

      %%sanity check:
      %Plot state estimation errors versus time
      figure(5),
      clf
      subplot(211)
      plot(tvec,xk_truehist(1,:)-mk_filt_hist(1,:),'b','LineWidth',3), hold on
      plot(tvec,2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
      plot(tvec,-2*sqrt(squeeze(Pk_filt_hist(1,1,:))'),'b--','LineWidth',2)
      subplot(212)
      plot(tvec,xk_truehist(2,:)-mk_filt_hist(2,:),'b','LineWidth',3), hold on
      plot(tvec,2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'b--','LineWidth',2)
      plot(tvec,-2*sqrt(squeeze(Pk_filt_hist(2,2,:))'),'b--','LineWidth',2)

  end
*/
}
