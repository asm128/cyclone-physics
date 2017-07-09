// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_RANDOM_H
#define CYCLONE_RANDOM_H

namespace cyclone {
	// Keeps track of one random stream: i.e. a seed and its output. This is used to get random numbers. Rather than a funcion, this allows there to be several streams of repeatable random numbers at the same time. Uses the RandRotB algorithm.
	class Random {
		// Internal mechanics
		int32_t									p1												= 0;
		int32_t									p2												= 0;
		uint32_t								Buffer[17]										= {};
	public:
		inline									Random											()											{ Seed(0);		}	// Creates a new random number stream with a seed based on timing data.
		inline									Random											(uint32_t seed)								{ Seed(seed);	}	// Creates a new random stream with the given seed.

		static inline	uint32_t				rotl											(uint32_t n, uint32_t r)					{ return (n << r) | (n >> (32 - r)); };	// left bitwise rotation
		static inline	uint32_t				rotr											(uint32_t n, uint32_t r)					{ return (n >> r) | (n << (32 - r)); };	// right bitwise rotation

		void									Seed											(uint32_t seed);			// Sets the seed value for the random stream.
		uint32_t								RandomBits										();							// Returns the next random bitstring from the stream. This is the fastest method.
		double									RandomReal										();							// Returns a random floating point number between 0 and 1.

		inline uint32_t							RandomInt										(uint32_t max)								{ return RandomBits() % max;					}	// Returns a random integer less than the given value.
		inline double							RandomReal										(double min, double max)					{ return RandomReal() * (max - min) + min;		}	// Returns a random floating point number between min and max.
		inline double							RandomReal										(double scale)								{ return RandomReal() * scale;					}	// Returns a random floating point number between 0 and scale.
		inline double							RandomBinomial									(double scale)								{ return (RandomReal() - RandomReal()) * scale;	}	// Returns a random binomially distributed number between -scale and +scale.
		Vector3									RandomVector									(double scale);				// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f].
		Vector3									RandomVector									(const Vector3 &scale);		// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f], where scale is the corresponding component of the given vector.
		Vector3									RandomVector									(const Vector3 &min, const Vector3 &max);	// Returns a random vector in the cube defined by the given minimum and maximum vectors. The probability is uniformly distributed in this region.
		Vector3									RandomXZVector									(double scale);				// Returns a random vector where each component is binomially distributed in the range (-scale to scale) [mean = 0.0f], except the y coordinate which is zero.
		Quaternion								RandomQuaternion								();							// Returns a random orientation (i.e. normalized) quaternion.
	};
} // namespace cyclone

#endif // CYCLONE_BODY_H
