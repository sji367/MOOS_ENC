/************************************************************/
/*    NAME: Sam                                             */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA.h                                        */
/*    DATE:                                                 */
/************************************************************/

#ifndef A_HEADER
#define A_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_OA : public IvPBehavior {
public:
  BHV_OA(IvPDomain);
  ~BHV_OA() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();
  

protected: // Local Utility functions
  IvPFunction* buildFunctionWithZAIC();

protected: // Configuration parameters

protected: // State variables
  string m_obstacles, m_obs_info;
  double m_ASV_x, m_ASV_y, m_ASV_head;
  int m_num_obs; 
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_OA(domain);}
}
#endif
