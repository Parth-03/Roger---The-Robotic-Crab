/*************************************************************************/
/* File:        project9.c                                               */
/* Description: User project #9 - PONG                                   */
/* Date:        12-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

/************************************************************************/
int search_track(), stereo_observation(), chase_touch(), inv_arm_kinematics(), fwd_arm_kinematics();

Observation obs, temp;
double tempx[10], tempy[10];
double home, deltaT= 0.03, oldtime=0.0;
int heading=0,control=0, n=0, direction;

int setpoint_filter(roger)
Robot* roger;
{
  double x= roger->base_setpoint[0];
  double y= roger->base_setpoint[1];
  double lx, ly, rx, ry;
  fwd_arm_kinematics(roger, LEFT, &lx, &ly);
  fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
  
  if(heading==RIGHT)
  {
    if(x>-0.5 || x<-4.5 || lx>-0.2 || rx>-0.2 || ly<-4.8 || ly<-4.8)
      return FALSE;
  }
  else
  {
    if(x<0.5 || x>4.5 || lx<0.2 || rx<0.2 || ly>4.8 || ly>4.8)
      return FALSE;
  }
  if(y<-1.5 || y>1.5)
    return FALSE;
  
  return TRUE;
}

void retreat(roger, time)
Robot* roger;
double time;
{
  int return_state;
  
  roger->base_setpoint[0]= home;
  roger->base_setpoint[1]= 0.0;
  //if(roger->base_position[0]==home || roger->base_position[1]==0)
    roger->base_setpoint[THETA]= atan2(roger->base_position[1]- 0.0, roger->base_position[0]- home);

  roger->arm_setpoint[LEFT][0]= (9.0*M_PI/10.0);
  roger->arm_setpoint[LEFT][1]= -(9.0*M_PI/10.0);
  roger->arm_setpoint[RIGHT][0]= -(9.0*M_PI/10.0);
  roger->arm_setpoint[RIGHT][1]= (9.0*M_PI/10.0);

  if(heading==RIGHT)
  {
    if(fabs(home- roger->base_position[0] >-0.5))
      search_track(roger, time);
  }
  else
  {
    if(fabs(home- roger->base_position[0] <0.5))
      search_track(roger, time);
  }
  
  

}
/*
int chase_touch_retreat()
Robot* roger;
double time;
{
  static int return_state= NO_REFERENCE;
  static int internal_state[2] = {NO_REFERENCE, NO_REFERENCE};

  internal_state[0]= chase_touch(roger, time);
  internal_state[1]= retreat(roger, time);
  int state= internal_state[0]+ (internal_state[1]*3);

  switch(state)
  {
}*/

void block(roger, time)
Robot* roger;
double time;
{
  /*if(control%2==0)
  {
    temp.push
    return 0;
  }
    //double slope = atan2(obs.pos[1]- temp.pos[1], obs.pos[0] - temp.pos[0]);
    //printf("%lf %lf\n", obs.pos[0], temp.pos[0]);
    if(obs.pos[0]- temp.pos[0]==0 && obs.pos[1]- temp.pos[1]==0)
    {
      //ball is still
      return 0;
    }
    else
    {*/
      //ball is moving
      /*double slope;
      for(int i=0; i<20; i++)
      {
        slope += (atan2(obs.pos[1]-tempy[i], obs.pos[0]- tempx[i]))*0.05;
      }   */
      double slope= (obs.pos[1] - temp.pos[1])/(obs.pos[0] - temp.pos[0]);   
      printf(" %lf %lf %lf %lf\n", obs.pos[1], obs.pos[0], temp.pos[1], temp.pos[0]);
      printf("%lf\n", slope);
      double y= (slope*(home-temp.pos[0]))+ temp.pos[1];
      if(y>1.5)
        roger-> base_setpoint[1]= 1.5;
      else if (y< -1.5)
      {
        roger->base_setpoint[1]= -1.5;
      }
      else
      {
        roger->base_setpoint[1]= y;
      }
      roger->base_setpoint[0]= home;
      //roger->base_setpoint[THETA]= atan2(roger->base_position[1]- obs, roger->base_position[0]- home);
     
      //int i=inv_arm_kinematics(roger, LEFT, obs.pos[0] /*+ (home/4.0)*/, obs.pos[1]);
      //int r=inv_arm_kinematics(roger, RIGHT, obs.pos[0] /*+ (home/4.0)*/, obs.pos[1]);
      printf("ball %lf\n", y);
      printf("%lf %lf %lf\n", roger->base_setpoint[0], roger->base_setpoint[1], roger->base_setpoint[THETA]);
       
}
  

void stop(roger)
Robot* roger;
{
  roger->base_setpoint[0]= roger->base_position[0];
  roger->base_setpoint[1]= roger->base_position[1];
}

int calculate_direction(roger, time)
Robot* roger;
double time;
{
  double result=0;
  //for(int i=0; i< 20; i++)
  //{
    if(obs.pos[0]< temp.pos[0])
      //result+= 0;
      return 0;
    else 
      //result+= 1;
      return 1;
  //}
  /*if((result*0.05)>0.7 || result==0)
    return 1;
  else
    return 0;*/
  
}

void offense(roger, time)
Robot* roger;
double time;
{
  //search_track(roger, time);
  /*if(time>2)
  {
    roger->base_setpoint[0]= obs.pos[0];
    roger->base_setpoint[1]= obs.pos[1];
    l= inv_arm_kinematics(roger, LEFT, obs.pos[0], obs.pos[1]);
    r= inv_arm_kinematics(roger, RIGHT, obs.pos[0], obs.pos[1]);
    printf("too much time\n");
  }*/
  if(setpoint_filter(roger))
  {
    if(chase_touch(roger, time, heading)==CONVERGED)
    {
      retreat(roger, time);
      oldtime=0;
      //printf("converged sdsdsdsdsdsdsdsds\n");
    }
      
      
  }
  else
  {
    retreat(roger);
  }
  
  
}  

void defense(roger, time)
Robot* roger;
double time;
{
  //block(roger, time);
  retreat(roger, time);
}

void project9_control(roger, time)
Robot *roger;
double time;
{
  if(control<1)
  {
    //printf("%lf\n", roger->base_position[THETA]);
    if(roger->base_position[THETA]> -0.5 && roger->base_position[THETA] <0.5)
    heading=1;
    else
    {
      heading=0;
    }
    oldtime=0;
  }

  if(heading==RIGHT)
  {
    home= -4.5;
    //return_state=0;
  }
  else
  {
    home= 4.5;
    //return_state=1; 
  }

  stereo_observation(roger, &obs, time);
  //search_track(roger, time);
  /*if(temp==0)
    temp=obs;*/
  /*if(n<20)
  {
    tempx[n] = obs.pos[0];
    tempy[n] = obs.pos[1];
    n+=1;
  }*/
  //printf("%lf\n",fmod(time,deltaT));
  /*if(obs.pos[0]>temp.pos[0])
    direction= 1;
  else
    direction= 0;
  printf("%lf %lf\n", obs.pos[0], temp.pos[0]);*/
  if(heading==RIGHT)
  {
    if(obs.pos[0] <0 && roger->base_position[0] <0)
    {
      if(n<50)
        retreat(roger, time);
      else
        offense(roger, time);
      n+=1;
      //if(n>=20)
      //{
        //printf("%lf \n", (fmod(time,deltaT)));

      /*  
      if(fmod(time,deltaT)<0.001)
      {
        
        direction =calculate_direction(roger, time);
        printf("direction %d\n", direction);
        if(direction==0)
        {
          if(n==1)
          {
            block(roger, time);
            printf("left blocking\n");
          }
          n+=1;
          
          //printf("direction %d\n", direction);
          //offense(roger, time);
        }
        else
        {
          offense(roger, time);
          n=0;
          printf("left attacking\n");
        }
        //n=0;
        //oldtime=time;
        temp=obs;
      //}
      }
      //printf("%lf %lf\n", roger->base_setpoint[0], roger->base_setpoint[1]);
      */
    }
    
    else
    {
      retreat(roger, time);
      n=0;
      //printf("left retreating\n");
    }
    
    //printf("%lf %lf %lf %lf %lf\n", time, roger->base_position[THETA], distance, distance_l, distance_r);
  }
  else
  {
    if(obs.pos[0] >0 && roger->base_position[0] >0)
    {
      if(n<50)
        retreat(roger, time);
      else
        offense(roger, time);
      n+=1;

      /*
      if(fmod(time,deltaT)<0.001)
      {
        direction =calculate_direction(roger, time);
        printf("direction %d\n", direction);
        if(direction==1)
        {
          
          if(n==1)
          {
            block(roger, time);
            printf("right blocking\n");
          }
          n+=1;
          //printf("right blocking\n");
          //printf("direction %d\n", direction);
        }
        else
        {
          offense(roger, time);
          n=0;
          printf("right attacking\n");
        }
        temp=obs;
      }
      */
    }
    else
    {
      retreat(roger, time);
      n=0;
      //printf("right retreating\n");
    }
  }
  control+=1;
  oldtime+=0.001;
  //printf("%lf %lf\n", roger->base_setpoint[0], roger->base_setpoint[1]);
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project9_enter_params() 
{
  printf("Project 9 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }

