/************************************************************/
/*    NAME: alonzo                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_rotate_lawnmower.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef rotate_lawnmower_HEADER
#define rotate_lawnmower_HEADER

#include <string>
#include "IvPBehavior.h"

//^_^
#include<XYPoint.h>
#include<string>
#include<sstream>
#include<vector>
#include<iostream>

class BHV_rotate_lawnmower : public IvPBehavior {
public:
  BHV_rotate_lawnmower(IvPDomain);
  ~BHV_rotate_lawnmower() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

  //^_^  variables

  double m_start_x;  
  double m_start_y;
  double m_h;       
  double m_d;
  double m_resolution;
  double m_osx;
  double m_osy;
  double m_theta;
  double m_w;
  int counter;//for case
  double m_arrival_radius;
  XYPoint m_nextpt;

  struct Data {
    double x;
    double y;
  };
  vector<struct Data> view_point; // it has to be placed after the struct, if not, error!
  vector<struct Data>::iterator p; //test

  string m_name;
  string m_information;
  double m_distance;

  IvPFunction* buildFunctionWithZAIC();

  //function
  void MarkpMarine(double x, double y);
  void setting_params(string msg);
  void setting_path();
  void calculate_distance();
  bool check_opregion(double x, double y);
  double StringToDouble(string msg);


protected: // Local Utility functions

protected: // Configuration parameters

protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_rotate_lawnmower(domain);}
}
#endif
