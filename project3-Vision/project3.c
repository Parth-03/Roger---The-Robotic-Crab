/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
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

int average_red_pixel(roger, ul, ur, time)
Robot* roger;
double *ul, *ur, time; 
{
  *ul=0.0, *ur=0.0;
   double theta_error;
  for(int k=0; k<NEYES; k++)
  {
    theta_error= roger->eyes_setpoint[k] - roger->eye_theta[k];
    int start=-1, end =-1;
    for(int i=0; i<NPIXELS; i++)
    {
      if(roger->image[k][i][RED_CHANNEL]== 255)
      {
        //estimate coordinates ul, ur
        if(start<0)
          start=i,end=i;
        else
          end+=1;
        continue;
      }
      if(start>=0)
        break;
    }
    if(k==LEFT)
      *ul=(start+end)/2.0;
    if(k==RIGHT)
      *ur=(start+end)/2.0;
    //printf("%lf %lf\n", time, theta_error);
  }
  if(ul>0 || ur>0)
    return(TRUE);
  else return(FALSE);
}
  

void project3_control(roger, time)
Robot* roger;
double time;
{ 
  double ul, ur;
  if(average_red_pixel(roger, &ul, &ur) == TRUE)
  {
    roger->eyes_setpoint[LEFT]= roger->eye_theta[LEFT] + atan2((ul-63.5), FOCAL_LENGTH);
    roger->eyes_setpoint[RIGHT]= roger->eye_theta[RIGHT] + atan2((ur-63.5), FOCAL_LENGTH);
    //printf("%lf \n", time);
  }
}

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project3_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }
