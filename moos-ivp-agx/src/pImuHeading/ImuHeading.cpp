/************************************************************/
/*    NAME: logan                                            */
/*    ORGN: MIT                                             */
/*    FILE: ImuHeading.cpp                                        */
/*    DATE: 2021/05/22                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ImuHeading.h"
#include "XYVector.h"
using namespace std;

//---------------------------------------------------------
// Constructor

ImuHeading::ImuHeading()
{
  m_osx = 0.0;
  m_osy = 0.0;
  m_imu_heading = 0.0;
  m_active = true;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ImuHeading::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "NAV_X")
     {
        m_osx = dval;
     }
     else if(key == "NAV_Y")
     {
        m_osy = dval;
     }
     else if(key == "NAV_HEADING_IMU") //for report
     {
        m_imu_heading = dval;
     }
  }

  return UpdateMOOSVariables(NewMail);  // Automatically updates registered MOOS Vars
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ImuHeading::OnConnectToServer()
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
  AddActiveQueue("foo_callback", this, &ImuHeading::onMessageFoo);
  AddMessageRouteToActiveQueue("foo_callback", "FOO_IN");
#endif

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void ImuHeading::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
     Register("NAV_X", 0);
     Register("NAV_Y", 0);
     Register("NAV_HEADING_IMU", 0);

  RegisterMOOSVariables();
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ImuHeading::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Do your thing here!
  ShowCompassHeading();


  PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ImuHeading::OnStartUp()
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

bool ImuHeading:onMessageFoo(CMOOSMsg& foo)
{
  // do something with foo

  // update outgoing message (FOO_OUT in this case)
  // SetMOOSVar("foo_msg", new_value, m_curr_time);

  return(true);
}
#endif

//------------------------------------------------------------
// Procedure: buildReport()

bool ImuHeading::buildReport()
{
  m_msgs << "pImuHeading Report\n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}



//------------------------------------------------------------
// Procedure: ShowCompassHeading
//   Purpose: Using VIEW_VECTOR to show compass heading on pMarineViewer

void ImuHeading::ShowCompassHeading()
{
        XYVector vector(m_osx, m_osy, 5, m_imu_heading);
        vector.set_active(m_active);
        vector.set_label("hdg");
        //vector.set_color("fill", "orange");
        vector.set_vertex_size(5);
        vector.set_edge_size(5);
        vector.set_edge_color("green");
        vector.setHeadSize(2);
        string str = vector.get_spec();
        m_Comms.Notify("VIEW_VECTOR", str);
}

