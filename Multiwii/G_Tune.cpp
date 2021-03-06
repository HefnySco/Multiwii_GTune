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
//  version 1.0.3:
				a. separate skip_time be axis.
				b. skip when rc stick is not neutral [+-Value]
				(a & b) are some techniques used by CrashZero in his code ... http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2?id=705844%3ABlogPost%3A1745293&page=3#comments
//	version 1.0.4:
				a. adding wobble detection.		
//	version 1.0.5:
				a. more accurate wobble detection		
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

// RC sticks over which calculation is skipped as sticks will corrupt the calculations.
#define ZERO_RC_STICKS		12			// (conf.pid[PIDALT].P8 ) 
// number of cycles that are skipped between each ZERO_PID execution.
#define TIME_SKIP	10					// conf.pid[PIDLEVEL].D8
// value over which I is incremented.
#define AVGERAGE_I_BOUNDARY	 15			// conf.pid[PIDALT].D8
// Number of I that are averaged together. These values are read with in active ZERO_PID execution. i.e. I is ignored during time_skip is active
#define I_CYCLE	100						// conf.pid[PIDALT].I8
// values for incrementing and decrementing P
#define High_Enough _1 		10			// conf.pid[PIDLEVEL].P8
#define High_Enough _1		15			// conf.pid[PIDLEVEL].I8


// Max P & I based on multiwii
#define MAX_TUNED_P	200
#define MAX_TUNED_I	100

int16_t time_skip[3] = {0,0,0}; // skip need to be multiples of three. as we enter here there times for 1 measure as we have three axis.
int16_t	I_Counter[3] = {0,0,0};
int8_t	Wobble_LastErrorSign[3] = {0,0,0};	//1 Positive, -1 Negative , 0 NA
int16_t Wobble_Rotation[3][2] = {{0,0},{0,0},{0,0}};
uint16_t Wobble_Counter[3]={0,0,0};
uint16_t Wobble_WaveLength[3]={0,0,0};
int16_t Wobble_time_skip[3] = {0,0,0};
bool I_Am_Called = false;

void init_ZEROPID()
{
	for (int i=0;i < 3;++i)
	{
		OrgPID[i].P8 = conf.pid[i].P8;
		OrgPID[i].I8 = conf.pid[i].I8;
		OrgPID[i].D8 = conf.pid[i].D8;
		// init using current PIDs
		//conf.pid[i].P8 =200;
		//conf.pid[i].I8 =0;
		//conf.pid[i].D8 =0;
		AvgPID[i].P8=0;
		AvgPID[i].I8=0;
		AvgPID[i].D8=0;
	}
	
}


void save_ZEROPID()
{
	if (I_Am_Called != true) return ;
	
	for (int i=0;i < 3;++i)
	{
			conf.pid[i].P8 = AvgPID[i].P8;
			conf.pid[i].I8 = AvgPID[i].I8;
			conf.pid[i].D8 = AvgPID[i].D8;
	}
}


void calculate_Wobble (uint8_t Axis, int16_t Stick, int16_t Error)
{
	int16_t  Errordiv16;
	
	// Skip if stick is not neutral.
	if (abs(Stick) > (conf.pid[PIDALT].P8 ))
	{   // skip if stick is non zero
		
		Wobble_time_skip[Axis]=conf.pid[PIDLEVEL].D8;
		return ;
	}
	
	Errordiv16 = Error / 4;
	// Wobble Detection
	if (Errordiv16 > 0)
	{  // from - to +
		
		if (Wobble_LastErrorSign[Axis]!=1)
		{   // complete wave length. time to measure and take action.
			Wobble_WaveLength[Axis]=Wobble_Counter[Axis];
		
			if (Wobble_time_skip[Axis] > 0)
			{  // stop changing P each time... leave time to allow updated P to take effect.
				Wobble_time_skip[Axis] -=1;
			}				
			else
			{   
				Wobble_time_skip[Axis]=conf.pid[PIDLEVEL].D8;
				// Wobble Detection
				//if (((abs( Wobble_Rotation[0][0] - Wobble_Rotation[0][1]) * Wobble_WaveLength[Axis]) > conf.pid[PIDNAVR].I8 << 2))
				if (((abs( Wobble_Rotation[0][0] - Wobble_Rotation[0][1]) >  conf.pid[PIDNAVR].D8) && (Wobble_WaveLength[Axis]) >  conf.pid[PIDNAVR].I8))
				{
					if (Axis==0) LEDPIN_OFF;
					//if ( conf.pid[Axis].P8 > 0)	{conf.pid[Axis].P8  -=1;}
				}
				else
				{
					if (Axis==0) LEDPIN_ON;
				}
		
			}
			if (Axis ==0)
			{
				debug[1]= Wobble_WaveLength[Axis];
				debug[2]= Wobble_Rotation[Axis][0] - Wobble_Rotation[Axis][1];
				debug[3]= abs( Wobble_Rotation[Axis][0] - Wobble_Rotation[Axis][1]) * Wobble_WaveLength[Axis];
			}
			
			Wobble_Rotation[Axis][0]=0;
			Wobble_Rotation[Axis][1]=0;
			Wobble_Counter[Axis]=0;
		}
		
		Wobble_Counter[Axis] +=1;
		
		if (Wobble_Rotation[Axis][0] < Errordiv16)
		{
			Wobble_Rotation[Axis][0] = Errordiv16;
			if (Axis ==0)  debug[0] = Wobble_Rotation[0][0];
		}
		
		//Wobble_Rotation[Axis][0] += Error;
		Wobble_LastErrorSign[Axis] =1;	
		//debug[0] = Wobble_Rotation[0][0];	
	}	
	else if (Errordiv16 < 0)
	{  // from + to -
		Wobble_Counter[Axis] +=1;
		if (Wobble_Rotation[Axis][1] > Errordiv16)
		{
			Wobble_Rotation[Axis][1] = Errordiv16;
		}
		
		//Wobble_Rotation[Axis][1] += Error;
		Wobble_LastErrorSign[Axis] = -1;		
		//debug[1] = Wobble_Rotation[0][1];			
	}	
}
void calculate_ZEROPID (uint8_t Axis,  int16_t Stick, int16_t Error)
{
	int16_t diff_P;
	int16_t diff_I;
	int16_t  Errordiv16;
	I_Am_Called = true;
	
	// Skip if stick is not neutral.
	if (abs(Stick) > (conf.pid[PIDALT].P8 ))
	{   // skip if stick is non zero
		time_skip[Axis] = 50; // 100 ms 
		Wobble_time_skip[Axis]=50;
		return ;
	}
	
	Errordiv16 = Error / 4;
	// Wobble Detection
	if (Errordiv16 > 0)
	{  // from - to +
		if (Wobble_LastErrorSign[Axis]!=1)
		{   
			// complete wave length.
			Wobble_WaveLength[Axis]=Wobble_Counter[Axis];
					   	
			if (Wobble_time_skip[Axis] > 0)
			{  // stop changing P each time... leave time to allow updated P to take effect.
				Wobble_time_skip[Axis] -=1;
			}				
			else
			{   
				Wobble_time_skip[Axis]=conf.pid[PIDLEVEL].D8;
				// Wobble Detection
				//if ( > 2) && (Wobble_WaveLength[Axis] > 12))
				//if (((abs( Wobble_Rotation[0][0] - Wobble_Rotation[0][1]) * Wobble_WaveLength[Axis]) > conf.pid[PIDNAVR].I8 << 2))
				if (((abs( Wobble_Rotation[0][0] - Wobble_Rotation[0][1]) >  conf.pid[PIDNAVR].D8) && (Wobble_WaveLength[Axis]) >  conf.pid[PIDNAVR].I8)
				&& (conf.pid[PIDNAVR].I8 !=0) && (conf.pid[PIDNAVR].D8 !=0))
				{
					if (Axis==0) LEDPIN_OFF;
					if ( conf.pid[Axis].P8 > 0)	{conf.pid[Axis].P8  -=1;}
					if ( conf.pid[Axis].I8 >= 10)	{conf.pid[Axis].I8  -=10;} else {conf.pid[Axis].I8 =0;}
				}
				else
				{
					if (Axis==0) LEDPIN_ON;
				}
			}
			if (Axis ==0)
			{
				debug[1]= Wobble_WaveLength[Axis];
				debug[2]= Wobble_Rotation[Axis][0] - Wobble_Rotation[Axis][1];
				debug[3]=abs( Wobble_Rotation[Axis][0] - Wobble_Rotation[Axis][1]) * Wobble_WaveLength[Axis];
			}
			
			Wobble_Rotation[Axis][0]=0;
			Wobble_Rotation[Axis][1]=0;
			Wobble_Counter[Axis]=0;
		}
		
		Wobble_Counter[Axis] +=1;
		
		if (Wobble_Rotation[Axis][0] < Errordiv16)
		{   // wobble_rotation [0] keeps the highest value
			Wobble_Rotation[Axis][0] = Errordiv16;
			if (Axis ==0) debug[0] = Wobble_Rotation[0][0];
		}
		
		Wobble_LastErrorSign[Axis] =1;	
	}	
	else if (Errordiv16 < 0)
	{  // from + to -
		Wobble_Counter[Axis] +=1;
		if (Wobble_Rotation[Axis][1] > Errordiv16)
		{ // wobble_rotation [1] keeps the lowest value 
			Wobble_Rotation[Axis][1] = Errordiv16;
		}
		
		Wobble_LastErrorSign[Axis] = -1;		
	}			
	
	
	if (time_skip[Axis] > 0 )
	{
		time_skip[Axis] -=1;
		return ; // dont enter the loop.
	}
	time_skip[Axis] = TIME_SKIP; //conf.pid[PIDLEVEL].D8;
	
	
	// Handling I Part
	// Try to fly in Acro mode with I factor equal to Zero, 
	// you will find that if you tilt your try then take your hand of the sticks it will not get back, 
	// if you add some values to I-factor, it will tend to restore itself. 
	// And this is obvious because the I-component –i.e. the integration value not the I factor value- 
	// that was accumulated to overcome your stick need to be reduced back to zero. 
	// To have this effect I used a simple average variable to calculate the average value of gyro readings 
	// in a time window, and if the value is not Zero –higher or lower than a certain range- I 
	// is incremented otherwise it is decremented.
	if (I_Counter[Axis] > 0 )
	{
		// calculate average to get non-zero DC value.
		AvgError[Axis] = ((AvgError[Axis] + Error )) >> 1;
		I_Counter[Axis] -= 1;
	}
	else
	{   // Update I based on averages
		if (conf.pid[PIDALT].D8 !=0)
		{
			if (abs(AvgError[Axis]) > conf.pid[PIDALT].D8) 
			{
				if (conf.pid[Axis].I8 < MAX_TUNED_I)  {conf.pid[Axis].I8 +=1;}
			}
			else if(abs(AvgError[Axis]) <= conf.pid[PIDALT].D8) 
			{
				if (conf.pid[Axis].I8 > 0)  {conf.pid[Axis].I8 -=1;}
			}
		}
		// Reset Averages
		AvgError[Axis] =0;
	
		I_Counter[Axis] = conf.pid[PIDALT].I8;
	}
	
	
	// Main Model of ZERO_PID Algorithm
	if ((Error >= 0 && OldError[P_INDEX][Axis] >=0) || (Error <= 0 && OldError[P_INDEX][Axis] <=0))	
	{	// Same Sign ... Core Algorithm is here
		diff_P = (int16_t)(abs((int16_t)Error) - abs(OldError[P_INDEX][Axis]));
		
		
		if ((diff_P > conf.pid[PIDLEVEL].P8) && (conf.pid[PIDLEVEL].P8 !=0)) // Condition #1:
		{ // 2.1 speed increased -in either positive or negative-
			if (conf.pid[Axis].P8 < MAX_TUNED_P)  {conf.pid[Axis].P8 +=1;}
		}
		else if ((diff_P < -conf.pid[PIDLEVEL].I8) && (conf.pid[PIDLEVEL].I8 !=0)) // Condition #2:		
		{ // 2.2 we are catching up now.
			if ( conf.pid[Axis].P8 > 0)	{conf.pid[Axis].P8  -=1;}
		}
	}

	
	
	// This condition to save final PID result from corruption due to landing 
	// and hitting ground. We change the PID for all throttle values to allow flying.
	// but dont save values except of valid flying conditions.
	// This also helps when you land, it reads the values before landing not after landing.
	if ((rcCommand[THROTTLE] > MINTHROTTLE + 250))
	{
	  AvgPID[Axis].P8 = conf.pid[Axis].P8; 
	  AvgPID[Axis].I8 = conf.pid[Axis].I8; 
	}
	
	//if (Axis==0)
	//{
	//	debug[0]=Error;
	//	debug[1]=OldError[P_INDEX][Axis];
	//	debug[2]= AvgPID[Axis].P8;
	//}
	
	// P & I for PITCH
	OldError[P_INDEX][Axis] = (int16_t)Error; 
	
}




#endif