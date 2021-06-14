/************************************************************/
/*    NAME: logan                                            */
/*    ORGN: MIT                                             */
/*    FILE: SourceLocalization.cpp                                        */
/*    DATE: 2021/06/11                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SourceLocalization.h"
#include <XYPoint.h>
#include <cmath>

using namespace std;

//---------------------------------------------------------
// Constructor

SourceLocalization::SourceLocalization()
{
	m_count = 0.0;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SourceLocalization::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "FOO")
      cout << "great!";

    else if(key == "NAV_X_HERON")
    {
       m_osx_heron = dval;
    }
    else if(key == "NAV_Y_HERON")
    {
       m_osy_heron = dval;
    }
    else if(key == "NAV_HEADING_HERON")
    {
       cout << "get !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
       m_hdg_heron = dval;
    }
    else if(key == "TDOA_ANGLE_HERON")
    {
       m_tdoa_angle_heron = dval;
    }
    else if(key == "NAV_X_DUCKIEBOAT")
    {
       m_osx_duckieboat = dval;
    }
    else if(key == "NAV_Y_DUCKIEBOAT")
    {
       m_osy_duckieboat = dval;
    }
    else if(key == "NAV_HEADING_DUCKIEBOAT")
    {
       m_hdg_duckieboat = dval;
    }
    else if(key == "TDOA_ANGLE_DUCKIEBOAT")
    {
       m_tdoa_angle_duckieboat = dval;
    }

  }



  return UpdateMOOSVariables(NewMail);  // Automatically updates registered MOOS Vars
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SourceLocalization::OnConnectToServer()
{
#if 0 // Keep this for messages that do not require a callback but just a simple read
  AddMOOSVariable("bar_msg", "BAR_IN", "BAR_OUT", 0);
  // incoming BAR_IN is automatically updated thru UpdateMOOSVariables();
  // to get the latest value of BAR_OUT, call :
  // double d = GetMOOSVar("bar_msg")->GetDoubleVal(); // if value is double
  // string s = GetMOOSVar("bar_msg")->GetStringVal(); // if value is string
#endif

#if 0 // Keep this as an example for callbacks
  AddMOOSVariable("foo_msg", "FOO_IN", "FOO_OUT", 0); // foo_msg is a local name
  AddActiveQueue("foo_callback", this, &SourceLocalization::onMessageFoo);
  AddMessageRouteToActiveQueue("foo_callback", "FOO_IN");
#endif

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SourceLocalization::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register("NAV_X_HERON", 0);
  Register("NAV_Y_HERON", 0);
  Register("NAV_HEADING_HERON", 0);
  Register("TDOA_ANGLE_HERON", 0);
  Register("NAV_Y_DUCKIEBOAT", 0);
  Register("NAV_X_DUCKIEBOAT", 0);
  Register("NAV_HEADING_DUCKIEBOAT", 0);
  Register("TDOA_ANGLE_DUCKIEBOAT", 0);

  RegisterMOOSVariables();
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SourceLocalization::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Do your thing here!

  // Assume 2 vehicle (0,0) & (80,0)
  // tdoa heading

  // 2. Calculate another possible angle
  double tdoa11 = m_tdoa_angle_heron;
  double tdoa21 = m_tdoa_angle_duckieboat;
  double tdoa12 = another_tdoa_angle(m_tdoa_angle_heron);
  double tdoa22 = another_tdoa_angle(m_tdoa_angle_duckieboat);
  cout << "tdoa heron         : " << m_tdoa_angle_heron << endl;
  cout << "nav heron          : " << m_hdg_heron << endl;


  // Calculate tdoa bearing in moos angle
  double theta11 = tdoa_to_moos_angle(m_hdg_heron, tdoa11);
  double theta21 = tdoa_to_moos_angle(m_hdg_duckieboat, tdoa21);
  double theta12 = tdoa_to_moos_angle(m_hdg_heron, tdoa12);
  double theta22 = tdoa_to_moos_angle(m_hdg_duckieboat, tdoa22);

  cout << "tdoa + nav to moos : " << theta11 << endl;
  //cout << "11: " << theta11 << endl;
  //cout << "12: " << theta12 << endl;
  //cout << "21: " << theta21 << endl;
  //cout << "22: " << theta22 << endl;

  // Transform moos angle in order to calculate m
  theta11 = theta_transform(theta11);
  theta12 = theta_transform(theta12);
  theta21 = theta_transform(theta21);
  theta22 = theta_transform(theta22);

  // 3. Calculate m & k
  double m1, m2, k1, k2;
  get_m_k(m_osx_heron, m_osy_heron, theta11, m1, k1);
  get_m_k(m_osx_duckieboat, m_osx_duckieboat, theta21, m2, k2); // here y

  //Cramer formula
  double result_x, result_y;
  cramer_formula(m1, m2, k1, k2, result_x, result_y);
  //cout << "coordinate : (" <<result_x << "," << result_y << ")" << endl;

  // Show the location of the sound source
  XYPoint point(50.0, 50.0);
  point.set_label("Sound");
  point.set_color("vertex", "blue");
  point.set_param("vertex_size", "3");
  string spec = point.get_spec();
  Notify("VIEW_POINT", spec);

  //XYPoint point2(50.0, 50.0);
  //point2.set_label("2");
  //point2.set_color("vertex", "green");
  //point2.set_param("vertex_size", "3");
  //spec = point2.get_spec();
  //Notify("VIEW_POINT", spec);

  //XYPoint test;
  //test.set_vx(50.0);
  //test.set_vy(50.0);
  //test.set_label("Sound");
  //Notify("VIEW_POINT", test.get_spec());


  PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SourceLocalization::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  else {
    STRING_LIST::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string orig  = *p;
      string line  = *p;
      string param = toupper(biteStringX(line, '='));
      string value = line;

      bool handled = false;
      if(param == "FOO") {
        handled = true;
      }
      else if(param == "BAR") {
        handled = true;
      }

      if(!handled)
        reportUnhandledConfigWarning(orig);
    }

  }

  registerVariables();
  return(true);
}

#if 0
//------------------------------------------------------------
// Procedure: onMessageFoo() callback

bool SourceLocalization:onMessageFoo(CMOOSMsg& foo)
{
  // do something with foo

  // update outgoing message (FOO_OUT in this case)
  // SetMOOSVar("foo_msg", new_value, m_curr_time);

  return(true);
}
#endif

//------------------------------------------------------------
// Procedure: buildReport()

bool SourceLocalization::buildReport()
{
  m_msgs << "pSourceLocalization Report\n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}


//------------------------------------------------------------
// Procedure: tdoa_to_moos_angle()

double SourceLocalization::tdoa_to_moos_angle(double nav_hdg, double tdoa_angle)
{
//     combine nav_heading(absolute) & tdoa angle(relative) into moos angle(absolute)
//
//     tdoa      0              moos      0         
//               |                        |
//      -90      |      90        90      |      270
//         ----- A -----            ----- A -----    
//      -90      |      90        90      |      270        
//               |                        |            
//          -180   +180                  180        
// 
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

//------------------------------------------------------------
// Procedure: another_tdoa_angle()

double SourceLocalization::another_tdoa_angle(double tdoa_angle)
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

//------------------------------------------------------------
// Procedure: theta_transform()

double SourceLocalization::theta_transform(double theta)
{
//
//    before      0             after      90
//                |                        |
//        90      |      270      180      |       0
//          ----- A -----            ----- A -----
//        90      |      270      180      |      360
//                |                        |
//               180                      270
//
        theta = theta + 90.0;
        if(theta > 360.0)
        {
                theta = theta - 360.0;
        }
        return theta;
}

//------------------------------------------------------------
// Procedure: get_m_k()

void SourceLocalization::get_m_k(double x, double y, double theta, double &m, double &k)
{
  m = tan(theta*PI/180.0);
  cout << "m : " << m << endl;
  k = y - m * x;
  cout << "y : " << y << endl;
  cout << "m : " << m << endl;
  cout << "x : " << x << endl;
  cout << "k : " << k << endl;
}

//------------------------------------------------------------
// Procedure: cramer_formula()

void SourceLocalization::cramer_formula(double m1, double m2, double k1, double k2, double &x, double &y)
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
