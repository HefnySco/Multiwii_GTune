/*
	G_Tune Mode
	This is the multiwii implementation of ZERO-PID Algorithm
	http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
	The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com)

	You may use/modify this algorithm on your own risk, kindly refer to above link in any future distribution.
*/
/*
// version 1.0.0: MIN & MAX & Tuned Band
// version 1.0.1: 
				a. error is gyro reading not rc - gyro.
				b. OldError = Error no averaging.
				c. No Min MAX BOUNDRY
//	version 1.0.2:
				a. no boundaries
				b. I - Factor tune.
				c. time_skip
*/			
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "G_Tune.h"

#if defined (G_TUNE)


#define P_INDEX	0
#define I_INDEX	1
#define D_INDEX	2



#define TUNED_BAND	4
#define TUNED_DIFF	4	
#define MAX_TUNED_P	200
#define MAX_TUNED_I	100


void init_ZEROPID()
{
	for (int i=0;i < 3;++i)
	{
		OrgPID[i].P8 = conf.pid[i].P8;
		OrgPID[i].I8 = conf.pid[i].I8;
		OrgPID[i].D8 = conf.pid[i].D8;
		conf.pid[i].P8 =0;
		conf.pid[i].I8 =0;
		conf.pid[i].D8 =0;
		AvgPID[i].P8=0;
		AvgPID[i].I8=0;
		AvgPID[i].D8=0;
	}
	
}


void save_ZEROPID()
{
	for (int i=0;i < 3;++i)
	{
		conf.pid[i].P8 = AvgPID[i].P8;
		conf.pid[i].I8 = AvgPID[i].I8;
		conf.pid[i].D8 = AvgPID[i].D8;
	}
}

int16_t time_skip = 0; // skip need to be multiples of three. as we enter here there times for 1 measure as we have three axis.
int16_t	I_Counter = 0;
void calculate_ZEROPID (uint8_t Axis, int16_t Error)
{
	int16_t diff_P;
	int16_t diff_I;
	
	
	
	
	if (time_skip < conf.pid[PIDLEVEL].D8)
	{
		//debug[3]= time_skip;
		if (Axis == 0)
		{   // increase me each
			time_skip +=1;
		}
		return ; // dont enter the loop.
		
	}
	
	if (Axis == 0) // because the AXIS will be 1 then 2 then 0 because the last zero used to increase time_skip then return.
	{  // reset me with the third hit only.
		time_skip = 0;
	}
	
	// handle I 
	if (I_Counter < conf.pid[PIDALT].I8)
	{
		// calculate average to get non-zero DC value.
		AvgError[Axis] = (AvgError[Axis] + Error ) / 2;
	
		if (Axis == 0)
		{	// increase skip counter.
			I_Counter += 1;
		}
	}
	else
	{   // Update I based on averages
		if (abs(AvgError[Axis]) > conf.pid[PIDALT].D8) 
		{
			if (conf.pid[Axis].I8 < MAX_TUNED_I)  {conf.pid[Axis].I8 +=1;}
		}
		else if(AvgError[Axis] <= conf.pid[PIDALT].D8) 
		{
			if (conf.pid[Axis].I8 > 0)  {conf.pid[Axis].I8 -=1;}
		}
		// Reset Averages
		AvgError[Axis] =0;
	
		if (Axis == 0)
		{
			I_Counter = 0;
		}		
	}
	
	

	if ((Error >= 0 && OldError[P_INDEX][Axis] >=0) || (Error <= 0 && OldError[P_INDEX][Axis] <=0))	
	{	// Same Sign ... Core Algorithm is here
		diff_P = (int16_t)(abs((int16_t)Error) - abs(OldError[P_INDEX][Axis]));
		
		
		if (diff_P > conf.pid[PIDLEVEL].P8) // Condition #1:
		{ // 2.1 speed increased -in either positive or negative-
			if (conf.pid[Axis].P8 < MAX_TUNED_P)  {conf.pid[Axis].P8 +=1;}
		}
		else if (diff_P < -conf.pid[PIDLEVEL].I8) // Condition #2:		
		{ // 2.2 we are catching up now.
			if ( conf.pid[Axis].P8 > 0)	{conf.pid[Axis].P8  -=1;}
			
		}
	}
/*
	else if (Error > 0)  
	{   // from - to +
		diff_P = (int16_t)(abs((int16_t)Error) + abs(OldError[P_INDEX][Axis]));
		// Nothing to Do Now
	}	
	else if (Error < 0)	
	{  // from + to - 
		diff_P = (int16_t)(abs((int16_t)Error) + abs(OldError[P_INDEX][Axis]));
		// Nothing to Do Now
	}			
*/	
	
	// this condition to save final PID result from corruption due to landing and hitting ground.			
	if ((rcCommand[THROTTLE] > MINTHROTTLE + 250))
	{
	  AvgPID[Axis].P8 = conf.pid[Axis].P8; 
	  AvgPID[Axis].I8 = conf.pid[Axis].I8; 
	}
	
	if (Axis==0)
	{
		debug[0]=Error;
		debug[1]=OldError[P_INDEX][Axis];
		debug[2]= AvgPID[Axis].P8;
	}
	
	// P & I for PITCH
	OldError[P_INDEX][Axis] = (int16_t)Error; 
	
}




#endif