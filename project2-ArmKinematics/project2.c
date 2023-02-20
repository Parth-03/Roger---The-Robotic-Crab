/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/
double refx, refy;// for plotting

void fwd_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double *x, *y;
{
  //x= l1c1 + l2c12 (c12= c1c2 - s1s2 which is the identity c(1+2))
  //same for y but with sin
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  *x= L_ARM1*cos(roger->arm_theta[limb][0]) + L_ARM2*(cos(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]));
  *y= L_ARM1*sin(roger->arm_theta[limb][0]) + L_ARM2*(sin(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]));
  //printf("%lf %lf\n", roger->arm_setpoint[limb][0]-x, roger->arm_setpoint[limb][1]-y);
  ref_b[0] = *x;
  ref_b[1] = *y;
  ref_b[2] = 0.0;
  ref_b[3] = 1.0;
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  construct_wTb(roger->base_position, wTb);
  matrix_mult(4, 4, wTb, 1, ref_b, ref_w);
  *x= ref_w[0];
  *y= ref_w[1];
}

int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  refx= x;
  refy= y;

  // input (x,y) is in world frame coordinates - map it into the base frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  x=ref_b[X];
  y=ref_b[Y];

  r2= (x*x)+ (y*y);// r2= x2 + y2
  c2= (r2 - (L_ARM1*L_ARM1) - (L_ARM2*L_ARM2))/(2*L_ARM1*L_ARM2);// c2= (r2- l1^2 - l2^2)/ 2l1l2  

  if((c2>= -1.0) && (c2<= 1.0))
  {
    s2_plus= sqrt(1-(c2*c2));
    s2_minus= -s2_plus;

    theta2_plus = atan2(s2_plus,c2);
    theta2_minus = atan2(s2_minus,c2);

    k1 = L_ARM1 + (L_ARM2*c2);
    k2_plus = L_ARM2*s2_plus;
    k2_minus = L_ARM2*s2_minus;

    alpha_plus= atan2(k2_plus,k1);
    alpha_minus= atan2(k2_minus,k1);

    theta1_plus= atan2(y,x) - alpha_plus;
    theta1_minus= atan2(y,x) - alpha_minus;

    if(limb==LEFT)
    {
      roger->arm_setpoint[LEFT][0]= theta1_minus;
      roger->arm_setpoint[LEFT][1]= theta2_minus;
    }
    else
    {
      roger->arm_setpoint[RIGHT][0]= theta1_plus;
      roger->arm_setpoint[RIGHT][1]= theta2_plus;
    }
    //printf("%lf\n", roger->arm_setpoint[limb][0]);
    return TRUE;
  }
  //printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", x, y);
  return FALSE;
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{
  double x,y; 
  fwd_arm_kinematics(roger, LEFT, &x, &y);
  //printf("%lf %lf %lf\n",time, x-(refx-ARM_OFFSET), refy-y);
}


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
  printf("Project 6 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }


