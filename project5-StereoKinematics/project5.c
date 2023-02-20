/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
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
#include "matrix_math.h"

Observation obs;

int stereo_observation(roger, obs, time)
Robot* roger;
Observation* obs;
double time;
{ 
  double ur, ul;
  double gammaL, gammaR, lambdaL, jacobian;
  double wTb[4][4], ref_b[4], ref_w[4], rot[2][2], J_wT[2][2], J_b[2][2], J_w[2][2];
  int average_red_pixel();

  if(average_red_pixel(roger, &ul, &ur))
  {
    gammaL= roger->eye_theta[LEFT] + atan2((ul-63.5), FOCAL_LENGTH);
    gammaR= roger->eye_theta[RIGHT] + atan2((ur-63.5), FOCAL_LENGTH);

    if(gammaR - gammaL ==0.0)
      lambdaL= jacobian= 20.0;
    else
    {
      lambdaL= (2*BASELINE*cos(gammaR))/sin(gammaR - gammaL);
      jacobian= (2*BASELINE)/(sin(gammaR - gammaL)*sin(gammaR - gammaL));
    }
    
    ref_b[0]= lambdaL* cos(gammaL);
    ref_b[1]= BASELINE + lambdaL* sin(gammaL);
    ref_b[2]= 0.0;
    ref_b[3]= 1.0;

    construct_wTb(roger->base_position, wTb);
    matrix_mult(4, 4, wTb, 1, ref_b, ref_w);

    obs->pos[0]= ref_w[0];
    obs->pos[1]= ref_w[1];

    rot[0][0]= wTb[0][0];
    rot[0][1]= wTb[0][1];
    rot[1][0]= wTb[1][0];
    rot[1][1]= wTb[1][1];

    //stereoJJT(roger, ur, ul, J_b);
    J_b[0][0]= jacobian*(cos(gammaR)*cos(gammaR));
    J_b[0][1]= -(jacobian*(cos(gammaL)*cos(gammaL)));
    J_b[1][0]= jacobian*(sin(gammaR)*cos(gammaR));
    J_b[1][1]= -(jacobian*(sin(gammaL)*cos(gammaL)));

    //matrix_transpose(2, 2, rot, J_wT);
    matrix_mult(2, 2, rot, 2, J_b, J_w);
    matrix_transpose(2, 2, J_w, J_wT);

    matrix_mult(2, 2, J_w, 2, J_wT, obs->cov);

    obs->cov[0][0]*= 0.001;
    obs->cov[0][1]*= 0.001;
    obs->cov[1][0]*= 0.001;
    obs->cov[1][1]*= 0.001;

    return True;	
  }
  return False;
}

/*void stereo_JJT(roger, ur, ul, JJT)
Robot* roger;
double ur, ul, JJT[2][2];
{ }
*/
void project5_control(roger, time)
Robot* roger;
double time;
{
  project3_control(roger, time); 
  stereo_observation(roger, &obs, time);
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  printf("Project 5 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{ 
  void draw_observation();
  draw_observation(roger,obs);
}
