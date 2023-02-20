/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int sample_gaze_direction(), average_red_pixel();
double recommended_setpoints[2][3];
double t=0;
double ctr=0;;

int search(roger, time)
Robot* roger;
double time;
{
  static double heading;
  double heading_error_base;
  static int return_state= NO_REFERENCE;
  if(return_state != TRANSIENT)
  {
    sample_gaze_direction(&heading);
    return_state= TRANSIENT;
  }
  else
  {
    heading_error_base= heading - roger->base_position[THETA];
    while(heading_error_base> M_PI)
    {
      heading_error_base-= 2.0*M_PI;
    }
    while(heading_error_base< -M_PI)
    {
      heading_error_base+= 2.0*M_PI;
    }
    
    recommended_setpoints[0][0]= heading;
    recommended_setpoints[0][1]= heading_error_base/2.0;
    recommended_setpoints[0][2]= heading_error_base/2.0;
    
    if(fabs(heading_error_base< 0.01))
      return_state= CONVERGED;   
  }
  return return_state;
}

int track(roger, time)
Robot* roger;
double time;
{
  double ul,ur, base_error, eye_error[2];
  static int return_state= NO_REFERENCE;

  if(average_red_pixel(roger, &ul, &ur))
  {
    eye_error[0]= atan2((ul-63.5), FOCAL_LENGTH);
    eye_error[1]= atan2((ur-63.5), FOCAL_LENGTH); 
    base_error= (roger->eye_theta[0]+ roger->eye_theta[1] + eye_error[0] + eye_error[1])/2.0;

    recommended_setpoints[1][0]= roger->base_position[THETA] + base_error;
    recommended_setpoints[1][1]= roger->eye_theta[0]+ eye_error[0];
    recommended_setpoints[1][2]= roger->eye_theta[1]+ eye_error[1];  

    if((fabs(eye_error[0])<0.1) && (fabs(eye_error[1])<0.1) && (fabs(base_error)<0.1))
      return_state= CONVERGED;
    else
      return_state= TRANSIENT;    
  }
  else
    return_state= NO_REFERENCE;
  return return_state;
}

int search_track(roger, time)
Robot* roger;
double time;
{
  static int return_state= NO_REFERENCE;
  static int internal_state[2] = {NO_REFERENCE, NO_REFERENCE};

  internal_state[0]= search(roger, time);
  internal_state[1]= track(roger, time);
  int state= internal_state[0]+ (internal_state[1]*3);

  switch(state)
  {
    case 0:
    case 1:
    case 2:
      //internal_state[0]= search(roger, time);
      roger->base_setpoint[THETA]= recommended_setpoints[0][0];
      roger->eyes_setpoint[0]= recommended_setpoints[0][1];
      roger->eyes_setpoint[1]= recommended_setpoints[0][2];
      return_state= TRANSIENT;
      break;
    case 3:
    case 4:
    case 5:
      //internal_state[1]= track(roger, time);
      roger->base_setpoint[THETA]= recommended_setpoints[1][0];
      roger->eyes_setpoint[0]= recommended_setpoints[1][1];
      roger->eyes_setpoint[1]= recommended_setpoints[1][2];
      return_state= TRANSIENT;
      break;
    case 6:
    case 7:
    case 8:
      return_state= CONVERGED;
      break;
    default:
      break;
  }
  return return_state;
}

void project4_control(roger, time)
Robot* roger;
double time;
{ 
  search_track(roger, time);
  if(t==0)
    t=time;
  if(t==time)
  {
    if(ctr==0)
      //printf("%lf %lf\n", time, (roger->base_setpoint[THETA]-roger->base_position[THETA]));
    ctr++; 
  }
  else
  {
    if(ctr>2)
    {
      ctr=0;
      t=time; 
    }
      
  }
  
  
}

/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }
