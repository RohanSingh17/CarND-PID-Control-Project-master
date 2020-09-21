#include <iostream>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp=Kp_;
	Ki=Ki_;
	Kd=Kd_;	
	
	p_error=0.0;
	i_error=0.0;
	d_error=0.0;
	std::cout << Kp <<"\t"<< Ki <<"\t"<< Kd<<"\n";
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   d_error=cte-p_error;
   p_error=cte;
   i_error+=cte;	
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (p_error*Kp+d_error*Kd+i_error*Ki);  // TODO: Add your total error calc here!
}

/*void PID::Twiddle(double cte){
	
/* 	vector<double> update_param;
	update_param.push_back(Kp/10.0);
	update_param.push_back(Kd/10.0);
	update_param.push_back(Ki/10.0); */
	
/*	double dp[] = {Kp/10.0,Kd/10,Ki/10};
	
	//double best_err=-TotalError();
	double best_err=cte*cte;
	
	double sum=0.0;
	
	for (int j=0;j<3;j++)
		sum+=dp[j];
	
	while (sum>0.02)
	{
		for (int i=0;i<3;i++)
		{
			Update_parameters(i,dp[i]);
			
			double err=TotalError();
			
			if (err < best_err)
			{
				best_err=err;
				dp[i]*=1.1;
			}
			else 
			{
				Update_parameters(i,-2*dp[i]);
				
				err=TotalError();
				if (err<best_err)
				{
					best_err=err;
					dp[i]*=1.1;
				}
				else
				{
					Update_parameters(i,dp[i]);
					dp[i]*=0.9;
				}
			}
			
			sum=0.0;
			for (int j=0;j<3;j++)
				sum+=dp[j];
		
		}		
	}
}

void PID::Update_parameters(int var, double update_par){
	if (var==0)
		Kp+=update_par;
	else if (var==1)
		Kd+=update_par;
	else
		Ki+=update_par;
}**/