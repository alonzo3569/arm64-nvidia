/************************************************************/
/*    NAME: logan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustControl.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <math.h> // for clamp
#include "MBUtils.h"
#include "ACTable.h"
#include "ThrustControl.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ThrustControl::ThrustControl()
{
    m_dMaxRudder = 0.0;
    m_dMaxThrust = 0.0;
    m_des_rudder = 0.0;
    m_des_thrust = 0.0;
    m_des_L = 0.0;
    m_des_R = 0.0;
}

//---------------------------------------------------------
// Destructor

ThrustControl::~ThrustControl()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ThrustControl::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

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

     else if(key == "DESIRED_RUDDER")
       m_des_rudder = msg.GetDouble();

     else if(key == "DESIRED_THRUST")
       m_des_thrust = msg.GetDouble();

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ThrustControl::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ThrustControl::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  thrustRudderToLR();
  Notify("ROS_THRUST_L", m_des_L);
  Notify("ROS_THRUST_R", m_des_R);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ThrustControl::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "max_rudder") {
      m_dMaxRudder = atof(value.c_str());
      handled = true;
    }
    else if(param == "max_thrust") {
      m_dMaxThrust = atof(value.c_str());
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void ThrustControl::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
   Register("DESIRED_RUDDER", 0);
   Register("DESIRED_THRUST", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ThrustControl::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}

//------------------------------------------------------------
// Procedure: clamp()
//   Purpose: Clamps the value of v between minv and maxv

double clamp(double v, double minv, double maxv)
{
    return min(maxv,max(minv, v));
}

//------------------------------------------------------------
// Procedure: thrustRudderToLR()

bool ThrustControl::thrustRudderToLR()
{
  // 1. Constrain Values
  //      DESIRED_RUDDER value to MAX_RUDDER
  //          - Anything more extreme than +/-50.0 is turn-in-place
  //      DESIRED_THRUST value to MAX_THRUST
  //          - Anything greater than +/-100.0% makes no sense
  double desiredRudder = clamp (m_des_rudder, (-1.0 * m_dMaxRudder), m_dMaxRudder);
  double desiredThrust = clamp (m_des_thrust, (-1.0 * MAX_THRUST), MAX_THRUST);

 // 2. Calculate turn
  //      - ADD rudder to left thrust
  //      - SUBTRACT rudder from right thrust
  double percentLeft  = desiredThrust + desiredRudder;
  double percentRight = desiredThrust - desiredRudder;

  // 3. Map desired thrust values to motor bounds
  //      - Range of DESIRED_THRUST: [-MAX_THRUST, MAX_THRUST]
  //      -          ...map to...
  //      - Range of valid thrust values: [-m_MaxThrustValue, m_MaxThrustValue]
  double fwdOrRevL   = (percentLeft  > 0.0) ? 1.0 : -1.0;
  double fwdOrRevR   = (percentRight > 0.0) ? 1.0 : -1.0;
  double pctThrustL  = fabs(percentLeft)  / MAX_THRUST;
  double pctThrustR  = fabs(percentRight) / MAX_THRUST;
  double mappedLeft  = pctThrustL * m_dMaxThrust * fwdOrRevL;
  double mappedRight = pctThrustR * m_dMaxThrust * fwdOrRevR;

  // 4. Deal with overages
  //      - Any value over m_MaxThrustValue gets subtracted from both sides equally
  //      - Constrain to [-m_MaxThrustValue, m_MaxThrustValue]
  double maxThrustNeg = -1.0 * m_dMaxThrust;
  if (mappedLeft  > m_dMaxThrust)
    mappedRight -= (mappedLeft  - m_dMaxThrust);
  if (mappedLeft  < maxThrustNeg)
    mappedRight -= (mappedLeft  + m_dMaxThrust);
  if (mappedRight > m_dMaxThrust)
    mappedLeft  -= (mappedRight - m_dMaxThrust);
  if (mappedRight < maxThrustNeg)
    mappedLeft  -= (mappedRight + m_dMaxThrust);

  m_des_L  = clamp (mappedLeft,  (-1.0 * m_dMaxThrust), m_dMaxThrust);
  m_des_R  = clamp (mappedRight, (-1.0 * m_dMaxThrust), m_dMaxThrust);
  return true;
}
