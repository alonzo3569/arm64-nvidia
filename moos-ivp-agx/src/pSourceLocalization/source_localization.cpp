#include<iostream>
#include <cmath>
#define PI 3.1415926
using namespace std;

/* Calculate location of the pinger 
 * using 2 vehicles' heading & GPS */

double tdoa_to_moos_angle(double nav_hdg, double tdoa_angle)
{
//     combine nav_heading(absolute) & tdoa angle(relative) into moos angle(absolute)
//
//     tdoa      0              moos      0         
//               |                        |
//      -90      |      90       270      |       90
//         ----- A -----            ----- A -----    
//      -90      |      90       270      |       90        
//               |                        |            
//          -180   +180                  180        
// 
	double tdoa_moos;
	if(tdoa_angle > 0.0)
	{
		tdoa_moos = nav_hdg + tdoa_angle ;
	}
	else
	{
		tdoa_moos = nav_hdg - abs(tdoa_angle);
	}

	if(tdoa_moos > 360.0)
	{
		tdoa_moos = tdoa_moos - 360.0;
	}
	else if(tdoa_moos < 0.0)
	{
		tdoa_moos = 360.0 - tdoa_moos;
	}
	return tdoa_moos;
}


double another_tdoa_angle(double tdoa_angle)
{
  double theta = abs(tdoa_angle);
  double another_angle;
  if(tdoa_angle < 0.0)
  {
	  another_angle = -(180.0 - theta);
  }
  else
  {
	  another_angle = 180.0 - theta;
  }
  cout << "another tdoa angle : " << another_angle << endl;
  return another_angle;
}

double theta_transform(double theta)
{
//
//    before      0             after      90          
//                |                        |
//       270      |       90      180      |       0
//          ----- A -----            ----- A -----   
//       270      |       90      180      |      360   
//                |                        |         
//               180                      270        
// 
	if(theta > 0.0 && theta < 90.0)
	{
		theta = -theta + 90.0;
	}
	else
	{
		theta = -theta + 450.0;
	}
	return theta;
}


void get_m_k(double x, double y, double theta, double &m, double &k)
{
  m = tan(theta*PI/180.0);
  cout << "m : " << m << endl;
  k = y - m * x;
  cout << "k : " << k << endl;
}

void cramer_formula(double m1, double m2, double k1, double k2, double &x, double &y)
{
  double delta = m2 - m1;
  cout << "delta : " << delta << endl;
  if(fabs(delta) < 1e-6)
  {
    cout << "No ans" << endl;
    return;
  }
      
  double delta_x = k1 - k2;
  double delta_y = m2 * k1 - m1 * k2;
  cout << "delta_x : " << delta_x << endl;
  cout << "delta_y : " << delta_y << endl;
  x = delta_x / delta;
  y = delta_y / delta;

  return;
}


void verify(double theta, double x, double result_x)
{
  if(theta > 0.0 && result_x - x > 0.0)
  {
    cout << "Correct location" << endl;
  }

  else if(theta < 0.0 && result_x - x < 0.0)
  {
    cout << "Correct location" << endl;
  }
  else
  {
    cout << "Wrong location" << endl;
  }
}


int main()
{
    // Input value
    //double x1, y1, theta1;
    //double x2, y2, theta2;
    double x1, y1, nav_hdg1, tdoa11;
    double x2, y2, nav_hdg2, tdoa21;
    nav_hdg1 = 180.0;
    nav_hdg2 = 180.0;
    tdoa11 = -135;
    tdoa21 = 135;
    x1 = 0.0;
    y1 = 0.0;
    x2 = 80.0;
    y2 = 0.0;

    // 2. Calculate another possible angle
    double tdoa12 = another_tdoa_angle(tdoa11); 
    double tdoa22 = another_tdoa_angle(tdoa21); 

    cout << "another angle : " << tdoa12 << endl;  

    // Calculate tdoa bearing in moos angle
    double theta11 = tdoa_to_moos_angle(nav_hdg1, tdoa11);
    double theta21 = tdoa_to_moos_angle(nav_hdg2, tdoa21);
    double theta12 = tdoa_to_moos_angle(nav_hdg1, tdoa12);
    double theta22 = tdoa_to_moos_angle(nav_hdg2, tdoa22);

    cout << "tdoa + nav heading to moos 11 : " << theta11 << endl; 
    cout << "12: " << theta12 << endl;
    cout << "21: " << theta21 << endl;
    cout << "22: " << theta22 << endl;

    // Transform moos angle in order to calculate m
    theta11 = theta_transform(theta11);
    theta12 = theta_transform(theta12);
    theta21 = theta_transform(theta21);
    theta22 = theta_transform(theta22);

    cout << "to tan theta 11 : " << theta11 << endl; 
    cout << "to tan theta 12 : " << theta12 << endl; 
    cout << "to tan theta 21 : " << theta21 << endl; 
    cout << "to tan theta 22 : " << theta22 << endl; 

    // 3. Calculate m & k
    double m1, m2, k1, k2;
    get_m_k(x1, y1, theta11, m1, k1);
    get_m_k(x2, y2, theta21, m2, k2);

    cout << "11: " << theta11 << endl;
    cout << "12: " << theta12 << endl;
    cout << "21: " << theta21 << endl;
    cout << "22: " << theta22 << endl;


    double theta[4] = {theta11, theta12, theta21, theta22};
    cout << "theta[0] : " << theta[0] << endl;
    cout << "theta[1] : " << theta[1] << endl;
    cout << "theta[2] : " << theta[2] << endl;
    cout << "theta[3] : " << theta[3] << endl;


    //Cramer formula
    double result_x, result_y;
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;
    //verify(theta1, x1, result_x);
    //double verify_theta = theta_transform(nav_hdg1);
    //double verify_m, verify_k;
    //get_m_k(x1, y1, verify_theta, verify_m, verify_k); // eq1
    // y = mx + k, assume x = 0, y = k

    // 3. Calculate m & k
    get_m_k(x1, y1, theta11, m1, k1);
    get_m_k(x2, y2, theta22, m2, k2);

    //Cramer formula
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;
 
    // 3. Calculate m & k
    get_m_k(x1, y1, theta12, m1, k1);
    get_m_k(x2, y2, theta21, m2, k2);

    //Cramer formula
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;

    // 3. Calculate m & k
    get_m_k(x1, y1, theta12, m1, k1);
    get_m_k(x2, y2, theta22, m2, k2);

    //Cramer formula
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;
    //verify(theta1, x1, result_x);
 

    return 0;
}


