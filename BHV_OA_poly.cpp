/************************************************************/
/*    NAME: Sam Reed                                        */
/*    ORGN: UNH                                             */
/*    FILE: BHV_OA.cpp                                      */
/*    DATE: June 2016                                       */
/************************************************************/

#ifdef _WIN32
#   define _USE_MATH_DEFINES
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif
 
#include <iterator>
#include <cstdlib>
#include <vector>
#include <sstream> // For stringstream
#include <string>
#include <cmath> // For sqrt and pow
#include <stdlib.h> // for atoi and atof (string to double/int)
#include <algorithm> // for max_element and sort

// MOOS Libraries
#include "XYPolygon.h"
#include "OF_Coupler.h"
#include "OF_Reflector.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "AngleUtils.h" // for RealAng
#include "ZAIC_Vector.h"
#include "BHV_OA_poly.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_OA_poly::BHV_OA_poly(IvPDomain gdomain) :
  IvPBehavior(gdomain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "ENC_OA");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("Next_WPT, Poly_Obs, NAV_SPEED, NAV_X, NAV_Y, NAV_HEAD");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_OA_poly::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "length")) {
    postWMessage(val);
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

void BHV_OA_poly::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_OA_poly::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_OA_poly::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_OA_poly::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_OA_poly::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_OA_poly::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_OA_poly::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_OA_poly::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  
  // Part 2a: Get information from the InfoBuffer
  bool ok1, ok2, ok3;
  m_obstacles = getBufferStringVal("Poly_Obs", ok1);
  m_WPT = getBufferStringVal("Next_WPT", ok2);
  m_speed = getBufferDoubleVal("NAV_SPEED", ok3);
  stringstream ss;
  // Check if there are new obstacles and speed and if there are, make a new IvPfunction
  if(!ok1) {
    postWMessage("No new obstacles info buffer.");
    return(0);
  }
  else if(!ok3)
    {
      postWMessage("Speed is not being defined.");
      return(0);
    }
  else
    {
      // Part 2b: Parse the obstacle information collected in the previous step
      // Parse the Waypoint Information
      if (ok2 and m_WPT!="first_point")
	{
	  vector<string> temp_WPT = parseString(m_WPT, ',');
	  m_WPT_x = (int)floor(strtod(temp_WPT[0].c_str(), NULL));
	  m_WPT_y = (int)floor(strtod(temp_WPT[1].c_str(), NULL));
	}
      // Seperate the individual pieces of the obstacle
      // The format is:
      //   ASV_X,ASV_Y,heading:# of Obstacles:t_lvl,type@min_ang,max_ang,min_dist_ang@min_ang_dist,max_ang_dist,min_dist!t_lvl,type@min_ang,max_ang,min_dist_ang@min_ang_dist,max_ang_dist,min_dist!...
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
	  ipf = buildZAIC_Vector();
	  //postWMessage("Building IvP");
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

/***************************************************************************/
/* This function parses the information on the polygon obstacles and then  */
/*   uses that information to create an IvPFunction using the ZAIC Vector  */
/*   tools. This function takes into account all polygons in the search    */
/*   area and does a linear interpolation between the minimum angle,       */
/*   maximum cost, and maximum angle points.                               */
/***************************************************************************/
IvPFunction *BHV_OA_poly::buildZAIC_Vector()
{
  IvPFunction *ivp_function = 0;
  double max_cost;
  stringstream ss,ss1,ss2, ss3;

  int maxutil = 100; // Need to look at the Waypoint behavior to see what their maximum utility is

  // This is a constant multiplier for the cost function that sets the prohibition zone such that the utility function is zero when the cost is greater than this constant. The radius of the prohibition zone will be directly proportional to the threat level of the object (t_lvl*multipler) 
  // Need to set the multiplier so that it is a function of the size and the current speed of the vessel
  double multiplier;
  double v_size = 4;
  multiplier = v_size/m_speed*4.5; 
  
  //  First seperate the obstacles from one another
  vector<string> info = parseString(m_obs_info, '!');
  
  // Declare which variable the domain is
  ZAIC_Vector head_zaic_v(m_domain, "course");

  // Used for the ZAIC_Vector function
  vector<double> domain_vals, range_vals;
  
  // Fill the array with maxiumum utility
  double OA_util[360];
  fill(OA_util,OA_util+360, maxutil);
  
  // This holds all the information on the polygon
  vector <poly_obs> all_poly_obs;

  for (unsigned int i=0;i<m_num_obs; i++)
    {
      // Initialize the stucture holding the information on the obstacle
      poly_obs obstacle;

      // Parse the individual obstacles
      vector<string> poly_info = parseString(info[i], '@');
      
      // Initialize attributes variable
      poly_attributes min_a, max_a, min_d;

      // General information on obstacle
      //    Type of obstacle and threat level
      vector<string> poly_gen_info = parseString(poly_info[0], ',');
	  
      // Type of obstacle and threat level
      obstacle.t_lvl = (int)floor(strtod(poly_gen_info[0].c_str(), NULL));
      obstacle.obs_type = poly_gen_info[1];

      // Information on the angles
      //    minimum angle, maximum angle, angle of minimum distance
      vector<string> poly_ang_info = parseString(poly_info[1], ',');
      double pnt1, pnt2;
      
      // min_angle
      pnt1 = (strtod(poly_ang_info[4].c_str(), NULL));
      pnt2 = (strtod(poly_ang_info[5].c_str(), NULL));
      obstacle.min_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
      // Max angle
      pnt1 = (strtod(poly_ang_info[0].c_str(), NULL));
      pnt2 = (strtod(poly_ang_info[1].c_str(), NULL));
      obstacle.max_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
      // Max Cost
      pnt1 = (strtod(poly_ang_info[2].c_str(), NULL));
      pnt2 = (strtod(poly_ang_info[3].c_str(), NULL));
      obstacle.min_dist.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
      
      ss.str(std::string());
      ss1.str(std::string());
      ss2.str(std::string());
      ss3.str(std::string());
      
      ss << m_num_obs;
      ss1 << obstacle.min_ang.ang;
      ss2 << obstacle.max_ang.ang;
      ss3 << obstacle.min_dist.ang;

      postMessage("Angle", ss.str() +" Obstacles - min: " + ss1.str()+", max: "+ss2.str()+", max_cost: "+ss3.str());

      
      // Information on the distance
      //    minimum angle distance, maximum angle distance, minimum distance
      vector<string> poly_dist_info = parseString(poly_info[2], ',');

      double d[3];
      for (int ii = 0; ii<3; ii++)
	{
	  d[ii] = (strtod(poly_dist_info[ii].c_str(), NULL));
	  if (d[ii]<1)
	    d[ii] = 1;
	}
      obstacle.min_ang.dist = d[0];
      obstacle.max_ang.dist = d[1];
      obstacle.min_dist.dist = d[2];

      // Determine the cost for each angle that you have information on
      obstacle.min_ang.cost = calc_cost(obstacle.t_lvl,obstacle.min_ang.dist,multiplier);
      obstacle.max_ang.cost = calc_cost(obstacle.t_lvl,obstacle.max_ang.dist,multiplier);
      obstacle.min_dist.cost = calc_cost(obstacle.t_lvl,obstacle.min_dist.dist,multiplier);

      // Calculate the slope and y intercepts
      // Deal with slope being inf --> set it = to large number
      if (obstacle.min_ang.ang == obstacle.min_dist.ang)
	{
	  obstacle.min_ang.m = 9999;
	}
      else
	obstacle.min_ang.m = (obstacle.min_ang.cost-obstacle.min_dist.cost)/(obstacle.min_ang.ang-obstacle.min_dist.ang);

      if (obstacle.max_ang.ang == obstacle.min_dist.ang)
	{
	  obstacle.max_ang.m = 9999;
	}
      else
	obstacle.max_ang.m = (obstacle.min_dist.cost-obstacle.max_ang.cost)/(obstacle.min_dist.ang-obstacle.max_ang.ang);

      obstacle.min_ang.b = obstacle.min_ang.cost - obstacle.min_ang.m*obstacle.min_ang.ang;
      obstacle.max_ang.b = obstacle.max_ang.cost - obstacle.max_ang.m*obstacle.max_ang.ang;
      
      double utility, c, cost;
      int cur_ang;
      string x,y;

      //postMessage("line_min", "y = "+doubleToString(obstacle.min_ang.m)+" x + "+doubleToString(obstacle.min_ang.m));

      // The buffer distance is to make sure that the ASV avoids the obstacle with some buffer
      int buffer_width = 20;
      // Make a larger buffer if the maximum cost (aka the minimum distance) is greater than 1 
      if (obstacle.min_dist.cost > 1)
	{
	  buffer_width += floor(pow(2*obstacle.min_dist.cost,2));
	  postMessage("buffer_w", buffer_width);
	}

      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> min angle to max cost
      
      for (double x1 = obstacle.min_ang.ang-buffer_width; x1<=obstacle.min_dist.ang; x1++)
	{
	  // Set a maximum threshold on the cost. If it is below this threshold, then decrease the cost by taking the cube of the cost*multiplier.
	  if (obstacle.min_ang.m == 9999)
	    cost = obstacle.min_dist.cost;
	  else
	    cost = (obstacle.min_ang.m*x1+obstacle.min_ang.b);
	  if (cost > 1)
	    {
	      x = doubleToString(m_ASV_x);
	      y = doubleToString(m_ASV_y);
	      postMessage("Cost", "x="+x+", y="+y+", Cost:"+doubleToString(cost));
	      c = 1;
	      //maxutil = pow(cost, .4)*maxutil;
	    }
	  else
	    {
	      c = pow(cost, 4);
	      maxutil = 100;
	    }
	  utility = maxutil*(1-c);	  

	  // Update the current angle
	  if (x1<0)
	    cur_ang = (int)floor(x1+360);
	  else if (x1>360)
	    cur_ang = (int)floor(x1-360);
	  else
	    cur_ang = (int)floor(x1);

	  // If the current utility value is less than the one for the gaussian window then store the one for the Gaussian window
	  if (utility<OA_util[cur_ang])
	    OA_util[cur_ang]=floor(utility);
	}
      
      // Reset max_util
      maxutil = 100;

      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> max cost to max angle
      for (double x2 = obstacle.min_dist.ang; x2<=obstacle.max_ang.ang+buffer_width; x2++)
	{
	  // Set a maximum threshold on the cost. If it is below this threshold, then decrease the cost by taking the cube of the cost*multiplier.
	  if (obstacle.max_ang.m == 9999)
	    cost = obstacle.min_dist.cost;
	  else
	    cost = (obstacle.max_ang.m*x2+obstacle.max_ang.b);
	  if (cost > 1)
	    {
	      c = 1;
	      //maxutil = pow(cost, .5)*maxutil;
	    }
	  else
	    {
	      c = pow(cost, 3);
	      maxutil = 100;
	    }

	  // Update the current angle
	  if (x2<0)
	    cur_ang = (int)floor(x2+360);
	  if (x2>360)
	    cur_ang = (int)floor(x2-360);
	  else
	    cur_ang = (int)floor(x2);

	  // If the current utility value is less than the previously stored value, store the new one
	  if (utility<OA_util[cur_ang])
	    OA_util[cur_ang]=floor(utility);
	}

      // Push the information on the indivual obstacle to a vector of type poly_obs
      all_poly_obs.push_back(obstacle);
    }

  // Set the values for the angle (domain) and utility (range)
  for (int iii = 0; iii<360; iii++)
    {
      domain_vals.push_back(iii);
      range_vals.push_back((int)floor(OA_util[iii]));
    }

  head_zaic_v.setDomainVals(domain_vals);
  head_zaic_v.setRangeVals(range_vals);

  // Clear the domain and range values
  domain_vals.clear();
  range_vals.clear();

  // Extract the IvP function
  ivp_function = head_zaic_v.extractIvPFunction();
  
  
  return(ivp_function);
}

double BHV_OA_poly::calc_cost(int t_lvl, double dist, double multiplier)
{
  double cost;
  stringstream ss;

  // Calculate original cost
  cost= t_lvl/dist*multiplier;

  return cost;
}
