// Because Cyclone is designed to work at either single or double precision, mathematical functions such as sqrt cannot be used in the source code or headers. 
// This file provides defines for the real number type and mathematical formulae that work on it.
// All the contents of this file need to be changed to compile Cyclone at a different precision.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include <float.h>
#include <cstdint>

#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#define REAL_MAX		DBL_MAX				// Defines the highest value for the real number. 
#define real_sqrt		sqrt				// Defines the precision of the square root operator. 
#define real_abs		fabs				// Defines the precision of the absolute magnitude operator. 
#define real_sin		sin					// Defines the precision of the sine operator. 
#define real_cos		cos					// Defines the precision of the cosine operator. 
#define real_exp		exp					// Defines the precision of the exponent operator. 
#define real_pow		pow					// Defines the precision of the power operator. 
#define real_fmod		fmod				// Defines the precision of the floating point modulo operator. 
#define real_epsilon	DBL_EPSILON			// Defines the number e on which 1+e == 1 **/
#define R_PI			3.14159265358979

#endif // CYCLONE_PRECISION_H
