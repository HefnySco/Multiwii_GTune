
#ifndef G_TUNE_H_
#define G_TUNE_H_


#if defined (G_TUNE)


static int16_t AvgError[3];
static int16_t OldError[2][3];
static pid_ OrgPID[3];
static pid_ AvgPID[3];

void init_ZEROPID();
void save_ZEROPID();
void calculate_ZEROPID (uint8_t Axis, int16_t Error);

#endif

#endif