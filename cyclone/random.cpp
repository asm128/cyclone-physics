// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "random.h"

#include <cstdlib>
#include <ctime>

using namespace cyclone;

					Random::Random					()											{ seed(0); }
					Random::Random					(uint32_t seed)								{ Random::seed(seed); }

void				Random::seed					(uint32_t s)								{
	if (s == 0) 
		s					= (uint32_t)clock();

	for (uint32_t i = 0; i < 17; i++) {			// Fill the buffer with some basic random numbers
		s					= s * 2891336453 + 1;	// Simple linear congruential generator
		buffer[i]			= s;
	}

	// Initialize pointers into the buffer
	p1					= 0;  
	p2					= 10;
}

uint32_t			Random::rotl					(uint32_t n, uint32_t r)					{ return (n << r) | (n >> (32 - r)); }
uint32_t			Random::rotr					(uint32_t n, uint32_t r)					{ return (n >> r) | (n << (32 - r)); }
uint32_t			Random::randomBits				()											{
	uint32_t				result = buffer[p1]				= rotl(buffer[p2], 13) + rotl(buffer[p1], 9);	// Rotate the buffer and store it back to itself

	// Rotate pointers
	if (--p1 < 0) p1		= 16;
	if (--p2 < 0) p2		= 16;

	return result;	// Return result
}

double				Random::randomReal				()											{
	uint32_t				bits				= randomBits();	// Get the random number
	// Set up a reinterpret structure for manipulation
	union {
		double					value;
		uint32_t				words[2];
	}						convert;

	// Now assign the bits to the words. This works by fixing the ieee sign and exponent bits (so that the size of the result is 1-2)
	// and using the bits to create the fraction part of the float. Note that bits are used more than once in this process.
	convert.words[0]	=  bits << 20; // Fill in the top 16 bits
	convert.words[1]	= (bits >> 12) | 0x3FF00000; // And the bottom 20
	return convert.value - 1.0;	// And return the value
}

uint32_t			Random::randomInt				(uint32_t max)								{ return randomBits() % max;				}
double				Random::randomReal				(double min, double max)					{ return randomReal() * (max-min) + min;	}
double				Random::randomReal				(double scale)								{ return randomReal() * scale;				}
double				Random::randomBinomial			(double scale)								{ return (randomReal()-randomReal())*scale; }
Quaternion			Random::randomQuaternion		()											{
	Quaternion				q								=
		{ randomReal()
		, randomReal()
		, randomReal()
		, randomReal()
		};
	q.normalise();
	return q;
}

Vector3				Random::randomVector			(double scale)								{
	return 
		{ randomBinomial(scale)
		, randomBinomial(scale)
		, randomBinomial(scale)
		};
}

Vector3				Random::randomXZVector			(double scale)								{
	return 
		{ randomBinomial(scale)
		, 0
		, randomBinomial(scale)
		};
}

Vector3				Random::randomVector			(const Vector3 &scale)						{
	return 
		{ randomBinomial(scale.x)
		, randomBinomial(scale.y)
		, randomBinomial(scale.z)
		};
}

Vector3				Random::randomVector			(const Vector3 &min, const Vector3 &max)	{
	return 
		{ randomReal(min.x, max.x)
		, randomReal(min.y, max.y)
		, randomReal(min.z, max.z)
		};
}
