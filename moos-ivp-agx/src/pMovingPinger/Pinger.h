/************************************************************/
/*    NAME: logan                                              */
/*    ORGN: MIT                                    */
/*    FILE: Pinger.h                                          */
/*    DATE: 2021/06/10                                      */
/************************************************************/

#ifndef Pinger_HEADER
#define Pinger_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
# include <time.h>

class Pinger : public AppCastingMOOSApp
{
 public:
  Pinger();
  ~Pinger() {};

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();

 protected: // Mail Callbacks
#if 0 // Keep this as an example for callbacks
  bool onMessageFoo(CMOOSMsg&);
#endif

 private: // Configuration variables

 private: // State variables

  double m_osx, m_osy;
  time_t m_toc;
  double m_pulse_range, m_pulse_duration;

  double m_osx_heron, m_osy_heron, m_hdg_heron;
  double m_osx_duckieboat, m_osy_duckieboat, m_hdg_duckieboat;

 protected: // member function
  double get_m(double, double, double, double);
  double get_tdoa_angle(double, double);
};

#endif
