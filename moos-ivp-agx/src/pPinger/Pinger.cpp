/************************************************************/
/*    NAME: logan                                            */
/*    ORGN: MIT                                             */
/*    FILE: Pinger.cpp                                        */
/*    DATE: 2021/06/10                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Pinger.h"
#include "XYRangePulse.h"
#include <XYPoint.h>
#include <math.h>       /* atan2 */

//#define PI 3.14159265
using namespace std;

//---------------------------------------------------------
// Constructor

Pinger::Pinger()
{
  m_osx = 0.0;
  m_osy = 0.0;
  m_toc = time(NULL);
  m_pulse_range = 20.0;
  m_pulse_duration = 4.0;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Pinger::OnNewMail(MOOSMSG_LIST &NewMail)
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
       m_hdg_heron = dval;
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

  } 




  return UpdateMOOSVariables(NewMail);  // Automatically updates registered MOOS Vars
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Pinger::OnConnectToServer()
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
  AddActiveQueue("foo_callback", this, &Pinger::onMessageFoo);
  AddMessageRouteToActiveQueue("foo_callback", "FOO_IN");
#endif

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void Pinger::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X_HERON", 0);
  Register("NAV_Y_HERON", 0);
  Register("NAV_HEADING_HERON", 0);
  Register("NAV_Y_DUCKIEBOAT", 0);
  Register("NAV_X_DUCKIEBOAT", 0);
  Register("NAV_HEADING_DUCKIEBOAT", 0);

  RegisterMOOSVariables();
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Pinger::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Do your thing here!

  // Pinger localtion
  m_osx = 40.0;
  m_osy = 40.0;
  time_t tic = time(NULL);
  double delta_t = difftime(tic, m_toc);

  //Show Pulse
  if(delta_t > 0 && delta_t < 2)
  {
    XYRangePulse pulse;
    pulse.set_x(m_osx);
    pulse.set_y(m_osy);
    pulse.set_label("bhv_pulse");
    pulse.set_rad(m_pulse_range);
    pulse.set_duration(m_pulse_duration);
    //pulse.set_time(toc);
    pulse.set_color("edge", "yellow");
    pulse.set_color("fill", "yellow");
    
    string spec = pulse.get_spec();
    Notify("VIEW_RANGE_PULSE", spec);
    
    m_toc = time(NULL);
  }

  XYPoint point(m_osx, m_osy);
  point.set_label("Pinger");
  point.set_color("vertex", "yellow");
  point.set_param("vertex_size", "3");
  string spec = point.get_spec();
  Notify("VIEW_POINT", spec);

  // Calculate vehicle's tdoa angle for pinger localization simualtion
  // Calaulate slop
  double m1 = get_m(m_osx_heron, m_osy_heron, m_osx, m_osy);
  double m2 = get_m(m_osx_duckieboat, m_osy_duckieboat, m_osx, m_osy);
  cout << "heron x           : " << m_osx_heron << endl;
  cout << "duckieboat x      : " << m_osx_duckieboat << endl;
  cout << "m2 in moos angle  : " << m2 << endl;
  cout << "m1 in moos angle  : " << m1 << endl;
  cout << "m2 in moos angle  : " << m2 << endl;
  
  //Calculate tdoa heading
  double tdoa_angle_v1 = get_tdoa_angle(m1, m_hdg_heron);
  double tdoa_angle_v2 = get_tdoa_angle(m2, m_hdg_duckieboat);
  cout << "tdoa v1 in moos angle  : " << tdoa_angle_v1 << endl;
  cout << "tdoa v2 in moos angle  : " << tdoa_angle_v2 << endl;
  Notify("TDOA_ANGLE_HERON", tdoa_angle_v1);
  Notify("TDOA_ANGLE_DUCKIEBOAT", tdoa_angle_v2);


  PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Pinger::OnStartUp()
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

bool Pinger:onMessageFoo(CMOOSMsg& foo)
{
  // do something with foo

  // update outgoing message (FOO_OUT in this case)
  // SetMOOSVar("foo_msg", new_value, m_curr_time);

  return(true);
}
#endif

//------------------------------------------------------------
// Procedure: buildReport()

bool Pinger::buildReport()
{
  m_msgs << "pPinger Report\n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}


//------------------------------------------------------------
// Procedure: get_m()
double Pinger::get_m(double x, double y, double pinger_x, double pinger_y)
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

//------------------------------------------------------------
// Procedure: get_tdoa_angle()
double Pinger::get_tdoa_angle(double m, double v1_nav_heading)
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
