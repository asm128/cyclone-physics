// Timing functions, frame management and profiling.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "timing.h"

#if (__APPLE__ || __unix)	// assume unix based OS
	#define TIMING_UNIX	1
	#include <stdlib.h>
	#include <sys/time.h>
	typedef unsigned long long	LONGLONG;
#else	// assume windows
	#define TIMING_WINDOWS	1
	#include <windows.h>
	#include <mmsystem.h>	// Import the high performance timer (c. 4ms).
	static double qpcFrequency;
#endif

static bool qpcFlag;		// Hold internal timing data for the performance counter.

// Internal time and clock access functions
uint64_t									systemTime									()						{
#if TIMING_UNIX
	struct timeval									tv;
	gettimeofday(&tv, 0);

	return tv.tv_sec * 1000 + tv.tv_usec/1000;
#else
	if(!qpcFlag)
		return (uint64_t)timeGetTime();
	static LONGLONG qpcMillisPerTick;
	QueryPerformanceCounter((LARGE_INTEGER*)&qpcMillisPerTick);
	return (uint32_t)(qpcMillisPerTick * qpcFrequency);
#endif
}

uint64_t									TimingData::getTime							()						{ return systemTime(); }

#if TIMING_WINDOWS
uint64_t									systemClock									()						{ return __rdtsc(); }
#endif

uint64_t									TimingData::getClock						()						{
#if TIMING_UNIX
	struct timeval									tv;
	gettimeofday(&tv, 0);

	return tv.tv_sec * 1000 + tv.tv_usec/1000;
#else
	return systemClock();
#endif
}

// Sets up the timing system and registers the performance timer.
void										initTime									()						{
#if TIMING_UNIX
	qpcFlag										= false;
#else
	LONGLONG										time;
	qpcFlag										= (QueryPerformanceFrequency((LARGE_INTEGER*)&time) > 0);
	if (qpcFlag)	// Check if we have access to the performance counter at this resolution.
		qpcFrequency								= 1000.0 / time;
#endif
}

static TimingData							*timingData									= NULL;	// Holds the global frame time that is passed around

// Retrieves the global frame info instance
TimingData&									TimingData::get								()						{ return (TimingData&)*timingData; }

// Updates the global frame information. Should be called once per frame.
void										TimingData::update							()						{
	if (!timingData) 
		return;

	if (!timingData->IsPaused)	// Advance the frame number.
		++timingData->FrameNumber;

	uint64_t										thisTime									= systemTime();	// Update the timing information.
	timingData->LastFrameDuration				= thisTime - timingData->LastFrameTimestamp;
	timingData->LastFrameTimestamp				= thisTime;

	// Update the tick information.
	uint64_t										thisClock									= getClock();
	timingData->LastFrameClockTicks				= thisClock - timingData->LastFrameClockstamp;
	timingData->LastFrameClockstamp				= thisClock;

	// Update the RWA frame rate if we are able to.
	if (timingData->FrameNumber > 1) {
		if (timingData->AverageFrameDuration <= 0)
			timingData->AverageFrameDuration			= (double)timingData->LastFrameDuration;
		else {
			// RWA over 100 frames.
			timingData->AverageFrameDuration			*= 0.99;
			timingData->AverageFrameDuration			+= 0.01 * (double)timingData->LastFrameDuration;
			timingData->FramesPerSecond					= (float)(1000.0/timingData->AverageFrameDuration);	// Invert to get FPS
		}
	}
}

void										TimingData::deinit							()						{
	if(timingData) {
		delete timingData;
		timingData									= 0;
	}
}

void										TimingData::init							()						{
	initTime();	// Set up the timing system.

	if (0 == timingData)		// Create the frame info object
		timingData									= new TimingData();

	// Set up the frame info structure.
	timingData->FrameNumber						= 0;
	timingData->LastFrameTimestamp				= systemTime();
	timingData->LastFrameDuration				= 0;
	timingData->LastFrameClockstamp				= getClock();
	timingData->LastFrameClockTicks				= 0;
	timingData->IsPaused						= false;
	timingData->AverageFrameDuration			= 0;
	timingData->FramesPerSecond					= 0;
}
