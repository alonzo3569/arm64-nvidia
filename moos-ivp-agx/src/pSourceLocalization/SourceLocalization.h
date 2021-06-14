/************************************************************/
/*    NAME: logan                                              */
/*    ORGN: MIT                                    */
/*    FILE: SourceLocalization.h                                          */
/*    DATE: 2021/06/11                                      */
/************************************************************/

#ifndef SourceLocalization_HEADER
#define SourceLocalization_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SourceLocalization : public AppCastingMOOSApp
{
 public:
  SourceLocalization();
  ~SourceLocalization() {};

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
  double m_osx_heron, m_osy_heron, m_hdg_heron, m_tdoa_angle_heron;
  double m_osx_duckieboat, m_osy_duckieboat, m_hdg_duckieboat, m_tdoa_angle_duckieboat;
  int m_count;
 protected:
  double tdoa_to_moos_angle(double, double);
  double another_tdoa_angle(double);
  double theta_transform(double);
  void get_m_k(double, double, double, double&, double&);
  void cramer_formula(double, double, double, double, double&, double&);

};

#endif
