/************************************************************/
/*    NAME: logan                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrustControl.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ThrustControl_HEADER
#define ThrustControl_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#define MAX_RUDDER      50.0
#define MAX_THRUST     100.0

class ThrustControl : public AppCastingMOOSApp
{
 public:
   ThrustControl();
   ~ThrustControl();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 protected:
   bool thrustRudderToLR();

 private: // Configuration variables
   double m_dMaxRudder;
   double m_dMaxThrust;

 private: // State variables
   double m_des_rudder;
   double m_des_thrust;
   double m_des_L;
   double m_des_R;
};

#endif 
