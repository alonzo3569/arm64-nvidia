#include<iostream>
#include <cmath>
#define PI 3.1415926
using namespace std;

/* Calculate location of the pinger 
 * using 2 vehicles' heading & GPS */

double tdoa_to_moos_angle(double nav_hdg, double tdoa_angle)
{
	double tdoa_moos;
	if(tdoa_angle > 0.0)
	{
		tdoa_moos = nav_hdg - tdoa_angle ;
	}
	else
	{
		tdoa_moos = nav_hdg + abs(tdoa_angle);
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



double theta_transform(double theta)
{
  // Input  : 0~180, 0~-180
  // Output : 0~360
  if(theta > 90.0 && theta < 180.0)
  {
    theta = -theta + 450;
  }
  else
  {
    theta = -theta + 90.0; 
  }
  return theta;
}

double another_tdoa_angle(double theta)
{
  theta = 360 -theta;
  cout << "another tdoa angle : " << theta << endl;
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
    double x1, y1, nav_hdg1, tdoa1;
    double x2, y2, nav_hdg2, tdoa2;
    nav_hdg1 = 180.0;
    nav_hdg2 = 180.0;
    tdoa1 = -135;
    tdoa2 = 135;
    x1 = 0.0;
    y1 = 0.0;
    x2 = 80.0;
    y2 = 0.0;

    // Calculate tdoa bearing in moos angle
    double theta11 = tdoa_to_moos_angle(nav_hdg1, tdoa1);
    double theta21 = tdoa_to_moos_angle(nav_hdg2, tdoa2);

    cout << theta11 << endl; 
    cout << theta21 << endl;
    //cout << theta_transform(theta1) << endl; 
    //cout << theta_transform(theta2) << endl;

    // 1. theta ttransform
    //double theta11 = theta_transform(theta1);
    //double theta21 = theta_transform(theta2);

    // 2. Calculate another possible angle
    double theta12 = another_tdoa_angle(theta11); 
    double theta22 = another_tdoa_angle(theta21); 
    
    // 3. Calculate m & k
    double m1, m2, k1, k2;
    get_m_k(x1, y1, theta11, m1, k1);
    get_m_k(x2, y2, theta21, m2, k2);

    //Cramer formula
    double result_x, result_y;
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;
    //verify(theta1, x1, result_x);

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


