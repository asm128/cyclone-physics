// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_RANDOM_H
#define CYCLONE_RANDOM_H

namespace cyclone {
	// Keeps track of one random stream: i.e. a seed and its output. This is used to get random numbers. Rather than a funcion, this allows there to be several streams of repeatable random numbers at the same time. Uses the RandRotB algorithm.
	class Random {
		// Internal mechanics
		int32_t									p1 = 0, p2										= 0;
		uint32_t								buffer[17]										= {};
	public:
		uint32_t								rotl											(uint32_t n, uint32_t r);	// left bitwise rotation
		uint32_t								rotr											(uint32_t n, uint32_t r);	// right bitwise rotation

												Random											();							// Creates a new random number stream with a seed based on timing data.
												Random											(uint32_t seed);			// Creates a new random stream with the given seed.

		void									seed											(uint32_t seed);			// Sets the seed value for the random stream.
		uint32_t								randomBits										();							// Returns the next random bitstring from the stream. This is the fastest method.
		double									randomReal										();							// Returns a random floating point number between 0 and 1.
		double									randomReal										(double scale);				// Returns a random floating point number between 0 and scale.
		double									randomReal										(double min, double max);	// Returns a random floating point number between min and max.
		uint32_t								randomInt										(uint32_t max);				// Returns a random integer less than the given value.
		double									randomBinomial									(double scale);				// Returns a random binomially distributed number between -scale and +scale.
		Vector3									randomVector									(double scale);				// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f].
		Vector3									randomVector									(const Vector3 &scale);		// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f], where scale is the corresponding component of the given vector.
		Vector3									randomVector									(const Vector3 &min, const Vector3 &max);	// Returns a random vector in the cube defined by the given minimum and maximum vectors. The probability is uniformly distributed in this region.
		Vector3									randomXZVector									(double scale);				// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f], except the y coordinate which is zero.
		Quaternion								randomQuaternion								();							// Returns a random orientation (i.e. normalized) quaternion.
	};
} // namespace cyclone

#endif // CYCLONE_BODY_H
