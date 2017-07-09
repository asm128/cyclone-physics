// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "random.h"

#include <cstdlib>
#include <ctime>

using namespace cyclone;

void				Random::Seed					(uint32_t s)								{
	if (s == 0) 
		s					= (uint32_t)clock();

	for (uint32_t i = 0; i < 17; i++) {			// Fill the buffer with some basic random numbers
		s					= s * 2891336453 + 1;	// Simple linear congruential generator
		Buffer[i]			= s;
	}

	// Initialize pointers into the buffer
	p1					= 0;  
	p2					= 10;
}

uint32_t			Random::RandomBits				()											{
	uint32_t				result = Buffer[p1]				= rotl(Buffer[p2], 13) + rotl(Buffer[p1], 9);	// Rotate the buffer and store it back to itself

	// Rotate pointers
	if (--p1 < 0) p1		= 16;
	if (--p2 < 0) p2		= 16;

	return result;	// Return result
}

double				Random::RandomReal				()											{
	uint32_t				bits				= RandomBits();	// Get the random number
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

Quaternion			Random::RandomQuaternion		()											{
	Quaternion				q								=
		{ RandomReal()
		, RandomReal()
		, RandomReal()
		, RandomReal()
		};
	q.normalise();
	return q;
}

Vector3				Random::RandomVector			(double scale)								{
	return 
		{ RandomBinomial(scale)
		, RandomBinomial(scale)
		, RandomBinomial(scale)
		};
}

Vector3				Random::RandomXZVector			(double scale)								{
	return 
		{ RandomBinomial(scale)
		, 0
		, RandomBinomial(scale)
		};
}

Vector3				Random::RandomVector			(const Vector3 &scale)						{
	return 
		{ RandomBinomial(scale.x)
		, RandomBinomial(scale.y)
		, RandomBinomial(scale.z)
		};
}

Vector3				Random::RandomVector			(const Vector3 &min, const Vector3 &max)	{
	return 
		{ RandomReal(min.x, max.x)
		, RandomReal(min.y, max.y)
		, RandomReal(min.z, max.z)
		};
}
