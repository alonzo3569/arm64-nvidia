/************************************************************/
/*    NAME: logan                                              */
/*    ORGN: MIT                                    */
/*    FILE: ImuHeading.h                                          */
/*    DATE: 2021/05/22                                      */
/************************************************************/

#ifndef ImuHeading_HEADER
#define ImuHeading_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ImuHeading : public AppCastingMOOSApp
{
 public:
  ImuHeading();
  ~ImuHeading() {};

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

 protected: //My function
  void ShowCompassHeading();


 protected: // Mail Callbacks
#if 0 // Keep this as an example for callbacks
  bool onMessageFoo(CMOOSMsg&);
#endif

 private: // Configuration variables

 private: // State variables
  double m_osx;
  double m_osy;
  double m_imu_heading;
  bool   m_active;

};

#endif
