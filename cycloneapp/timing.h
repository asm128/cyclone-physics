// Holds the timing system for the physics demos.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include <cstdint>

#ifndef CYCLONE_DEMO_TIMING_H
#define CYCLONE_DEMO_TIMING_H

// Represents all the information that the demo might need about the timing of the game: current time, fps, frame number, and so on. */
struct TimingData {
	uint32_t					FrameNumber								= 0;		// The current render frame. This simply increments. 
	uint32_t					LastFrameTimestamp						= 0;		// The timestamp when the last frame ended. Times are given in milliseconds since some undefined time. 
	uint32_t					LastFrameDuration						= 0;		// The duration of the last frame in milliseconds. 
	uint64_t					LastFrameClockstamp						= 0;		// The clockstamp of the end of the last frame.
	uint64_t					LastFrameClockTicks						= 0;		// The duration of the last frame in clock ticks. 
	bool						IsPaused								= false;	// Keeps track of whether the rendering is paused. 
	// Calculated data
	double						AverageFrameDuration					= 0;		// This is a recency weighted average of the frame time, calculated from frame durations. 
	float						FramesPerSecond							= 0;		// The reciprocal of the average frame duration giving the mean fps over a recency weighted average.

	static TimingData&			get										();			// Gets the global timing data object.
	static void					update									();			// Updates the timing system, should be called once per frame.
	static void					init									();			// Initialises the frame information system. Use the overall init function to set up all modules. */
	static void					deinit									();			// Deinitialises the frame information system.
	static uint32_t				getTime									();			// Gets the global system time, in the best resolution possible. Timing is in milliseconds.
	static unsigned long		getClock								();			// Gets the clock ticks since process start.

private:
	// These are private to stop instances being created: use get().
								TimingData								()							{}

								TimingData								(const TimingData &)		= delete;
	TimingData&					operator=								(const TimingData &)		= delete;
};

#endif // CYCLONE_DEMO_TIMING_H
