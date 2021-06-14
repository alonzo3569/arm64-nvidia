#include<iostream>
#include <cmath>  /* tan */
#define PI 3.1415926
using namespace std;

/* Calculate location of the pinger 
 * using 2 vehicles' heading & GPS */

double theta_transform(double theta)
{
  // Input  : 0~90, 0~-90
  // Output : 0~180
  if(theta < 0)
  {
    theta = -theta + 90;
  }
  return theta;
}

void cramer_formula(double m1, double m2, double k1, double k2, double &x, double &y)
{
  double delta = m2 - m1;
  cout << "delta : " << delta << endl;
  if(delta == 0.0)
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

int main()
{
    // Input value
    double x1, y1, theta1;
    double x2, y2, theta2;
    theta1 = 45;
    theta2 = -45;
    x1 = 0.0;
    y1 = 0.0;
    x2 = 80.0;
    y2 = 0.0;

    cout << theta_transform(theta1) << endl; 
    cout << theta_transform(theta2) << endl;

    // 1. theta ttransform
    theta1 = theta_transform(theta1);
    theta2 = theta_transform(theta2);

    // 2. Calculate m & k
    double m1 = tan(theta1*PI/180.0);
    double m2 = tan(theta2*PI/180.0);
    cout << m1 << endl;
    cout << m2 << endl;

    double k1 = y1 - m1 * x1;
    double k2 = y2 - m2 * x2;
    cout << k1 << endl;
    cout << k2 << endl;

    //Cramer formula
    double result_x, result_y;
    cramer_formula(m1, m2, k1, k2, result_x, result_y);
    cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;
    
    return 0;
}
