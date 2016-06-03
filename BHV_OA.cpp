/************************************************************/
/*    NAME: Sam                                             */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA.cpp                                      */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_PEAK.h"
#include "BHV_OA.h"
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element
#include "XYPolygon.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_OA::BHV_OA(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "default_name");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, Obstacles");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_OA::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_OA::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_OA::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_OA::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_OA::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_OA::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_OA::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_OA::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_OA::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  stringstream ss;
  // Part 1a: Get information from the InfoBuffer
  bool ok1;
  m_obstacles = getBufferStringVal("Obstacles", ok1);
  if(!ok1) {
    postWMessage("No new obstacles info buffer.");
    return(0);
  }
  else
    {

  // Part 1b: Parse the obstacle information collected in the previous step
  
  // Seperate the individual pieces of the obstacle
  // The format is:
  //   ASV_X,ASV_Y,heading:# of Obstacles:x,y,t_lvl,type!x,y,t_lvl,type!...
      vector<string> result = parseString(m_obstacles, ':');

      // Parse ASV info
      vector<string> ASV_info = parseString(result[0], ',');
  
      // Convert strings to doubles
      m_ASV_x = strtod(ASV_info[0].c_str(), NULL);
      m_ASV_y = strtod(ASV_info[1].c_str(), NULL);
      m_ASV_head = strtod(ASV_info[2].c_str(), NULL);

      // Parse the number of obstacles
      m_num_obs = (int)floor(strtod(result[1].c_str(), NULL));

      // Store the information on the obstacle
      if (result.size()==3)
	{
	  m_obs_info = result[2];
	  ipf = buildFunctionWithZAIC(); 
	}
    }
  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf){
    ipf->setPWT(m_priority_wt);
  }

  return(ipf);
}

// This only really works when there is only one obstacle in the search zone
IvPFunction *BHV_OA::buildFunctionWithZAIC()
{
  double obs_x, obs_y, max_cost, dist, util;
  string obs_type;
  int obs_t_lvl;
  vector<double> cost;
  vector<int> ang;
  stringstream ss, ss1, ss2, ss3, ss4;
  string str, poly, obs_pos;
  // This is a constant multiplier for the cost function that sets the prohibition zone such that the utility function is zero when the cost is greater than this constant. The radius of the prohibition zone will be directly proportional to the threat level of the object (t_lvl*multipler) 
  double multiplier; 
  // These are the values need to create a ZAIC
  int summit, peakwidth, basewidth, summitdelta, minutil, maxutil;

  basewidth = 20; // Arbitrarly picked 10 degrees on each side
  peakwidth = 180-basewidth;
  summitdelta = 1;
  maxutil = 100; // Need to look at the Waypoint behavior to see what their maximum utility is

  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  for (unsigned int i=0;i<info.size(); i++){
    // Parse the individual obstacles
    vector<string> ind_obs_info = parseString(info[i], ',');

    // Convert the strings to doubles and ints
    vector<string> x = parseString(ind_obs_info[0], '=');
    obs_x = strtod(x[1].c_str(), NULL); // Get rid of the 'x='
    vector<string> y = parseString(ind_obs_info[1], '=');
    obs_y = strtod(y[1].c_str(), NULL); // Get rid of the 'y='
    obs_t_lvl = (int)floor(strtod(ind_obs_info[2].c_str(), NULL));
    
    // Type of obstacle
    obs_type = ind_obs_info[3];

    // Calculate the angle and cost for the obstacle
    ang.push_back(relAng(m_ASV_x, m_ASV_y, obs_x, obs_y));
    dist = sqrt(pow(m_ASV_x-obs_x,2) +pow(m_ASV_y-obs_y,2));

    // Make sure you dont divide by zero - if it is less than 1, set it 
    //  equal to 1
    if (dist <1)
      dist = 1;
    cost.push_back(obs_t_lvl/dist);
  }
  multiplier = 4.5; // need to set this so that it is a function of the size and the current speed of the vessel
  ZAIC_PEAK head_zaic(m_domain, "course");
  

  poly = ",format=radial,radius=30,pts=3,edge_color=darkviolet,label=obs";
  max_cost = *max_element(cost.begin(), cost.end());
  // If the maximum cost is zero, then we dont want to create a ZAIC
  //   function describing the utility function
  if (max_cost != 0)
    {
      // ZAIC Components: summit, peakwidth, basewidth, summitdelta, 
      //   minutil, and maxutil  
      for (unsigned int ii=0;ii<cost.size(); ii++)
	{
	  if (cost[ii]==max_cost)
	    {
	      if (cost[ii] > 1/multiplier) 
		{
		  cost[ii] = 1/multiplier;
		}

	      minutil = (int)floor((1-cost[ii]*multiplier)*maxutil);
	      ss << (minutil);
	      ss1 << (ang[ii]);
	      str =  "Angle: " + ss1.str() + " Utility: " + ss.str();
	      postMessage("ENC_OA", str);

	      // Create an objective function using MOOS's ZAIC
	      summit = (ang[ii]+180)%360;
	      head_zaic.setParams(summit, peakwidth, basewidth, summitdelta, minutil, maxutil);
	      ss3 << obs_x;
	      ss4 << obs_y;
	      obs_pos = "x="+ss3.str()+",y="+ss4.str();
	      postMessage("VIEW_POLYGON", obs_pos+poly);
	    }
	  // If we are not at the end of the cost vector add a new
	  //  component to the ZAIC function
	  //if (ii<cost.size()-1)
	  //int index = head_zaic.addComponent();
	}
    }
  if (cost.empty())
    {
      obs_pos = "x=5000,y=5000";
      postMessage("VIEW_POLYGON", obs_pos+poly);
    }
  // If the maximum cost is zero create a utility function that doesnt
  //   is happy at any angle
  //else
  //  head_zaic.setParams(0, 7, 0, 0, max_util, max_util);
  
  head_zaic.setValueWrap(true); // Wrap around the heading axis
  head_zaic.setSummitInsist(true);
  
  // Set the IvP Function to take the maximum value if there is overlap
  bool take_the_max = true;

  IvPFunction *ipf = 0;
  if(head_zaic.stateOK()){
    ipf = head_zaic.extractIvPFunction(take_the_max);
  }
  else
    // Post warnings (if there are any) to the MOOSDB
    postWMessage(head_zaic.getWarnings());
  
  return(ipf);
}
