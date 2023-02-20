/*************************************************************************/
/* File:        project7.c                                               */
/* Description: User project #7 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int search_track(), inv_arm_kinematics(), stereo_observation();
void fwd_arm_kinematics(); //for plotting

Observation obs;
double recommended_setpoints[2], distance, distance_l, distance_r;

int chase(roger, time)
Robot* roger;
double time;
{
  double x_error, y_error, slope;
  static int return_state= NO_REFERENCE;
  static int internal_state[2] = {NO_REFERENCE, NO_REFERENCE};

  internal_state[0]= search_track(roger, time);
  internal_state[1]= stereo_observation(roger, &obs, time);
  int state= internal_state[0]+ (internal_state[1]*3);

  switch(state)
  {
    case 0:
    case 3:
      return_state= NO_REFERENCE;
      break;
    case 1:
    case 2:
      recommended_setpoints[0]= roger->base_position[0];
      recommended_setpoints[1]= roger->base_position[1];
      return_state= NO_REFERENCE;
      break;
    case 4:
    case 5:
      x_error= obs.pos[0]- roger->base_position[0];
      y_error= obs.pos[1]- roger->base_position[1];
      distance= sqrt(SQR(x_error) + SQR(y_error));
      if(distance < 0.5)
      {
        recommended_setpoints[0]= roger->base_position[0];
        recommended_setpoints[1]= roger->base_position[1];
        return_state= CONVERGED;
      }
      else
      {
        slope= atan2(y_error, x_error);
        /*if(time>2)
        {
          recommended_setpoints[0]= obs.pos[0];
          recommended_setpoints[1]=obs.pos[1];
          printf("too much time\n");
        }
        else
        {*/
        //printf("%lf\n", slope);
          recommended_setpoints[0]= /*obs.pos[0];*/roger->base_position[0] + ((distance-0.5)*cos(slope));
          recommended_setpoints[1]= /*obs.pos[1];*/roger->base_position[1] + ((distance-0.5)*sin(slope));
 
        //}
        
        return_state= TRANSIENT;
      }
      break;   
  }
  return return_state;  
}

int touch(roger, time, heading)
Robot* roger;
double time;
int heading;
{
  static int return_state= NO_REFERENCE;
  int l=0,r=0;
  if(heading==RIGHT)
  {
    if(roger->base_position[1] < obs.pos[1])
    {
      if(roger->base_position[1] < obs.pos[1]-0.4)
        l= inv_arm_kinematics(roger, LEFT, obs.pos[0]-0.2, obs.pos[1]-0.1);
    }
      
    else if(roger->base_position[1] > obs.pos[1])
    {
      if(roger->base_position[1] > obs.pos[1]+0.4)
        r= inv_arm_kinematics(roger, RIGHT, obs.pos[0]-0.2, obs.pos[1]+0.1);
    }
    /*else
    {
      l= inv_arm_kinematics(roger, LEFT, obs.pos[0]-0.1, obs.pos[1]-0.1);
      r= inv_arm_kinematics(roger, RIGHT, obs.pos[0]+0.1, obs.pos[1]+0.1);
    }*/
    
  }
  else
  {
    if(roger->base_position[1] > obs.pos[1])
    {
      if(roger->base_position[1]> obs.pos[1]+0.4)
        l = inv_arm_kinematics(roger, LEFT, obs.pos[0]+0.2, obs.pos[1]+0.1);
    }
      
    else if(roger->base_position[1] < obs.pos[1])
    {
      if(roger->base_position[1] < obs.pos[1]-0.4)
        r= inv_arm_kinematics(roger, RIGHT, obs.pos[0]+0.2, obs.pos[1]-0.1);
    }
      /*
    else
    {
      l = inv_arm_kinematics(roger, LEFT, obs.pos[0]+0.1, obs.pos[1]+0.1);
      r= inv_arm_kinematics(roger, RIGHT, obs.pos[0]+0.1, obs.pos[1]-0.1);
    }*/
    
  }
  
  double lx, ly, rx, ry;
  fwd_arm_kinematics(roger, LEFT, &lx, &ly);
  fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
  distance_l= sqrt(SQR(obs.pos[0]-lx) + SQR(obs.pos[1]-ly));
  distance_r= sqrt(SQR(obs.pos[0]-rx) + SQR(obs.pos[1]-ry));

  if(l || r)
  {
    if((fabs(roger->ext_force[LEFT][0])>0) || (fabs(roger->ext_force[LEFT][1])>0) || (fabs(roger->ext_force[RIGHT][0])>0) || (fabs(roger->ext_force[RIGHT][1])>0))
      return_state= CONVERGED;
    else
      return_state= TRANSIENT;    
  }
  else
    return_state= NO_REFERENCE;
  return return_state;
}

int chase_touch(roger, time, heading)
Robot* roger;
double time;
int heading;
{
  static int return_state= NO_REFERENCE;
  static int internal_state[2] = {NO_REFERENCE, NO_REFERENCE};

  internal_state[0]= chase(roger, time);
  internal_state[1]= touch(roger, time, heading);
  int state= internal_state[0]+ (internal_state[1]*3);

  switch(state)
  {
    case 0:
    case 3:
    case 6:
      //out of sight
      return_state = NO_REFERENCE;
      roger->arm_setpoint[LEFT][0]= (9.0*M_PI/10.0);
      roger->arm_setpoint[LEFT][1]= -(9.0*M_PI/10.0);
      roger->arm_setpoint[RIGHT][0]= -(9.0*M_PI/10.0);
      roger->arm_setpoint[RIGHT][1]= (9.0*M_PI/10.0);
      break;
    case 1:
      //out of reach
      return_state = TRANSIENT;
      roger->arm_setpoint[LEFT][0]= (9.0*M_PI/10.0);
      roger->arm_setpoint[LEFT][1]= -(9.0*M_PI/10.0);
      roger->arm_setpoint[RIGHT][0]= -(9.0*M_PI/10.0);
      roger->arm_setpoint[RIGHT][1]= (9.0*M_PI/10.0);
      roger->base_setpoint[0]= recommended_setpoints[0];
      roger->base_setpoint[1]= recommended_setpoints[1];
      break;
    case 2:
    case 4:
    case 5:
      return_state= TRANSIENT;
      break;
    case 7:
    case 8:
      return_state= CONVERGED;
      break;

  }
  //printf("%lf %lf %lf %lf %lf\n", time, roger->base_position[THETA], distance, distance_l, distance_r);
  return return_state;
}

void project7_control(roger, time)
Robot* roger;
double time;
{
  chase_touch(roger, time, 0);
  //printf("%lf %lf %lf %lf %lf\n", time, roger->base_position[THETA], distance, distance_l, distance_r);
}

/************************************************************************/
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  printf("Project 7 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{}
