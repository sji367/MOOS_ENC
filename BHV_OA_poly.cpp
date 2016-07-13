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
#include <cstring> // For memset
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
  // Initialize Variables
  m_obstacles_str = "";
  m_obs_info = "";
  m_WPT = "";
  m_ASV_x = 0;
  m_ASV_y = 0;
  m_ASV_head = 0;
  m_speed = 0;
  m_num_obs = 0;
  m_WPT_x = 0;
  m_WPT_y = 0;
  /*
  // Initialize struct
  m_obstacle.t_lvl = 0;
  m_obstacle.obs_type = "";
  // Min Angle
  m_obstacle.min_ang.ang = 0;
  m_obstacle.min_ang.cost = 0;
  m_obstacle.min_ang.dist = 0;
  m_obstacle.min_ang.m = 0;
  m_obstacle.min_ang.b = 0;
  // Max Angle
  m_obstacle.max_ang.ang = 0;
  m_obstacle.max_ang.cost = 0;
  m_obstacle.max_ang.dist = 0;
  m_obstacle.max_ang.m = 0;
  m_obstacle.max_ang.b = 0;
  // Min Distance
  m_obstacle.min_dist.ang = 0;
  m_obstacle.min_dist.cost = 0;
  m_obstacle.min_dist.dist = 0;
  m_obstacle.min_dist.m = 0;
  m_obstacle.min_dist.b = 0;
  */

  int m_num_obs, m_WPT_x, m_WPT_y;
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
  m_obstacles_str = getBufferStringVal("Poly_Obs", ok1);
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
      //   ASV_X,ASV_Y,heading:# of Obstacles:t_lvl,type@min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y@min_ang_dist,max_ang_dist,min_dist!t_lvl,type@min_ang_x,min_ang_y,min_dist_x,min_dist_y,max_ang_x,max_ang_y@min_ang_dist,max_ang_dist,min_dist...
      vector<string> result = parseString(m_obstacles_str, ':');

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
  IvPFunction *ivp_function;
  ivp_function= 0;
  double max_cost = 0;

  int maxutil = 100; // Need to look at the Waypoint behavior to see what their maximum utility is

  // This is a constant multiplier for the cost function that sets the prohibition zone such that the utility function is zero when the cost is greater than this constant. The radius of the prohibition zone will be directly proportional to the threat level of the object (t_lvl*multipler) 
  // Need to set the multiplier so that it is a function of the size and the current speed of the vessel
  double multiplier = 0;
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
  std::memset(OA_util, maxutil, 360);
  
  for (unsigned int i=0;i<m_num_obs; i++)
    {
      // Parse the individual obstacles
      vector<string> poly_info = parseString(info[i], '@');
      if (poly_info.size() ==3)
	{
	  // General information on obstacle
	  //    Type of obstacle and threat level
	  vector<string> poly_gen_info = parseString(poly_info[0], ',');
	  if (poly_gen_info.size()==2)
	    {	  
	      // Type of obstacle and threat level
	      m_obstacle.t_lvl = (int)floor(strtod(poly_gen_info[0].c_str(), NULL));
	      m_obstacle.obs_type = poly_gen_info[1];

	      // Information on the angles
	      //    minimum angle, maximum angle, angle of minimum distance
	      vector<string> poly_ang_info = parseString(poly_info[1], ',');
	      double pnt1 = 0;
	      double pnt2 = 0;
	      
	      if (poly_ang_info.size()==6)
		{
		  // min_angle
		  pnt1 = (strtod(poly_ang_info[4].c_str(), NULL));
		  pnt2 = (strtod(poly_ang_info[5].c_str(), NULL));
		  m_obstacle.min_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
		  // Max angle
		  pnt1 = (strtod(poly_ang_info[0].c_str(), NULL));
		  pnt2 = (strtod(poly_ang_info[1].c_str(), NULL));
		  m_obstacle.max_ang.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
		  // Max Cost
		  pnt1 = (strtod(poly_ang_info[2].c_str(), NULL));
		  pnt2 = (strtod(poly_ang_info[3].c_str(), NULL));
		  m_obstacle.min_dist.ang = (relAng(m_ASV_x, m_ASV_y, pnt1, pnt2));
      
		  // Information on the distance
		  //    minimum angle distance, maximum angle distance, minimum distance
		  vector<string> poly_dist_info = parseString(poly_info[2], ',');
		  if (poly_dist_info.size() == 3)
		    {
		      double d[3] = {0,0,0};
		      for (int ii = 0; ii<3; ii++)
			{
			  d[ii] = (strtod(poly_dist_info[ii].c_str(), NULL));

			  if (d[ii]<1)
			    {
			      d[ii] = 1;
			    }
			}
      
		      m_obstacle.min_ang.dist = d[0];
		      m_obstacle.max_ang.dist = d[1];
		      m_obstacle.min_dist.dist = d[2];

		      // Determine the cost for each angle that you have information on
		      m_obstacle.min_ang.cost = calc_cost(m_obstacle.t_lvl,m_obstacle.min_ang.dist,multiplier);
		      m_obstacle.max_ang.cost = calc_cost(m_obstacle.t_lvl,m_obstacle.max_ang.dist,multiplier);
		      m_obstacle.min_dist.cost = calc_cost(m_obstacle.t_lvl,m_obstacle.min_dist.dist,multiplier);

		      // Calculate the slope and y intercepts
		      // Deal with slope being inf --> set it = to large number
		      if (m_obstacle.min_ang.ang == m_obstacle.min_dist.ang)
			{
			  m_obstacle.min_ang.m = 9999;
			}
		      else
			m_obstacle.min_ang.m = (m_obstacle.min_ang.cost-m_obstacle.min_dist.cost)/(m_obstacle.min_ang.ang-m_obstacle.min_dist.ang);

		      if (m_obstacle.max_ang.ang == m_obstacle.min_dist.ang)
			{
			  m_obstacle.max_ang.m = 9999;
			}
		      else
			m_obstacle.max_ang.m = (m_obstacle.min_dist.cost-m_obstacle.max_ang.cost)/(m_obstacle.min_dist.ang-m_obstacle.max_ang.ang);

		      m_obstacle.min_ang.b = m_obstacle.min_ang.cost - m_obstacle.min_ang.m*m_obstacle.min_ang.ang;
		      m_obstacle.max_ang.b = m_obstacle.max_ang.cost - m_obstacle.max_ang.m*m_obstacle.max_ang.ang;
      
		      double utility = maxutil;
		      double c = 0;
		      double cost = 0;
		      int cur_ang = 0;
		      string x = "";
		      string y = "";
		      // The buffer distance is to make sure that the ASV avoids the obstacle with some buffer
		      int buffer_width = 20;
		      // Make a larger buffer if the maximum cost (aka the minimum distance) is greater than 1 
		      if (m_obstacle.min_dist.cost > 1)
			{
			  buffer_width += floor(pow(2*m_obstacle.min_dist.cost,2));
			  postMessage("buffer_w", buffer_width);
			}

		      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> min angle to max cost
      
		      for (double x1 = m_obstacle.min_ang.ang-buffer_width; x1<=m_obstacle.min_dist.ang; x1++)
			{
			  // Set a maximum threshold on the cost. If it is below this threshold, then decrease the cost by taking the cube of the cost*multiplier.
			  if (m_obstacle.min_ang.m == 9999)
			    cost = m_obstacle.min_dist.cost;
			  else
			    cost = (m_obstacle.min_ang.m*x1+m_obstacle.min_ang.b);
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
			      //maxutil = 100;
			    }
			  utility = maxutil*(1-c);

			  if (utility < 0)
			    utility =0;	  

			  // Update the current angle
			  cur_ang = (int)floor((int)x1%360);
			  if (cur_ang < 0)
			    cur_ang+=360;
			  postMessage("Cur_Angle", cur_ang);

			  // If the current utility value is less than the one for the gaussian window then store the one for the Gaussian window
			  if (utility<OA_util[cur_ang])
			    OA_util[cur_ang]=floor(utility);
			}
		    
		      // Reset max_util
		      maxutil = 100;

		      // This calculates the utility and stores that value if it is less than the current utility for all obstacles --> max cost to max angle
		      for (double x2 = m_obstacle.min_dist.ang; x2<=m_obstacle.max_ang.ang+buffer_width; x2++)
			{
			  // Set a maximum threshold on the cost. If it is below this threshold, then decrease the cost by taking the cube of the cost*multiplier.
			  if (m_obstacle.max_ang.m == 9999)
			    cost = m_obstacle.min_dist.cost;
			  else
			    cost = (m_obstacle.max_ang.m*x2+m_obstacle.max_ang.b);
			  if (cost > 1)
			    {
			      c = 1;
			      //maxutil = pow(cost, .5)*maxutil;
			    }
			  else
			    {
			      c = pow(cost, 3);
			      //maxutil = 100;
			    }

			  // Update the current angle
			  cur_ang = (int)floor((int)x2%360);
			  if (cur_ang < 0)
			    cur_ang+=360;

			  utility = maxutil*(1-c);

			  if (utility < 0)
			    utility =0;
			  
			  // If the current utility value is less than the previously stored value, store the new one
			  if (utility<OA_util[cur_ang])
			      OA_util[cur_ang]=floor(utility);
			}
		    }
		  else
		    cout << "Failed - poly_dist_info wrong size: "  << poly_dist_info.size()<< "--> Should be 3" << endl;
		}
	      else
		cout << "Failed - poly_ang_info wrong size: "  << poly_ang_info.size()<< "--> Should be 6" << endl;
	    }
	  else
	    cout << "Failed - poly_gen_info wrong size: "  << poly_gen_info.size()<< "--> Should be 2"<< endl;
	}
      else
	cout << "Failed - poly_info wrong size: "  << poly_info.size() << "--> Should be 3" << endl;
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
