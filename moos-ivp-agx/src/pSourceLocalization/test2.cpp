/* Calculate vehicle's toda heading
 * using pinger & vehicles' GPS */
#include<iostream>
#include <math.h>       /* atan2 */

#define PI 3.14159265
using namespace std;

double get_m(double x, double y, double pinger_x, double pinger_y)
{
	// atan2 output m range : [-pi,+pi] with respect to x axis
	// get_m funtion output : moos angle
//
//     atan2     90            moos      0         
//               |                       |
//      180      |      0        90      |      270
//         ----- A -----           ----- A -----    
//     -180      |      0        90      |      270        
//               |                       |            
//              -90                     180        
//  

	double delta_y = pinger_y - y;
	double delta_x = pinger_x - x;
	double m = atan2(delta_y, delta_x) * 180 / PI; // atan angle

	// Change atan2 to moos angle
	if(m > 90.0 && m < 180.0)
	{
		m = m - 90.0;
	}
	else
	{
		m = m + 270.0;
	}
	return m;
}


double get_tdoa_angle(double m, double v1_nav_heading)
{
	double theta = v1_nav_heading - m;
	if(theta > 180.0)
	{
		theta = theta - 360.0;
	}
	else if(theta < -180.0)
	{
		theta = theta + 360.0;
	}
	return theta;
}


int main()
{
	// Init 
	double pinger_x = 40;
	double pinger_y = -40;
	double v1_x = 0;
	double v1_y = 0;
	double v2_x = 80;
	double v2_y = 0;

	double v1_nav_heading = 0.0;
	double v2_nav_heading = 0.0;

	// Calcualate slop
	double m1 = get_m(v1_x, v1_y, pinger_x, pinger_y);
	double m2 = get_m(v2_x, v2_y, pinger_x, pinger_y);
	cout << "m1 in moos angle  : " << m1 << endl;
	cout << "m2 in moos angle  : " << m2 << endl;

	//Calculate tdoa heading
	double tdoa_angle_v1 = get_tdoa_angle(m1, v1_nav_heading);
	double tdoa_angle_v2 = get_tdoa_angle(m2, v2_nav_heading);
	cout << "tdoa v1 in moos angle  : " << tdoa_angle_v1 << endl;
	cout << "tdoa v2 in moos angle  : " << tdoa_angle_v2 << endl;
		

	return 0;
}
