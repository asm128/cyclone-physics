// The core contains utility functions, helpers and a basic set of mathematical types.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
//
// -- Cyclone Reference
//
// Cyclone is a general purpose and robust system for real-time simulation of rigid bodies. The library was designed to be used in computer games, but may be applicable to other areas of simulation or research.
//
// -- About This Document
//
// This documentation contains detailed reference to every aspect of the Cyclone library.
//
// -- Contents
//
// Use the navigation system on the left side of the page to view the documentation. The navigation tool uses JavaScript, and requires a version 4 browser or above.
//
// All the publically accessible functions and classes of Cyclone are provided in a set of header files. These, and their contents, can be browsed from the File List section.
//
// Cyclone is contained in a single namespace, cyclone. 
// Its contents can be viewed in the Compound List section. 
// The Class Hierarchy section provides an alternative way to navigate these classes. 
// The Graphical Class Hierarchy provides an overview of class inheritance.
// The Compound List section gives an alphabetic list of all symbols in the library, including method names and functions.
//
// -- Graphs
//
// Most of the documentation contains detailed graphical representations of the file and class dependencies. These diagrams are clickable, and provide the fastest mechanism for browsing the documentation. Each diagram is followed by a link to a help file giving a legend.
//
// -- Using Cyclone
//
// To set up:
//
// - Create a set of instances of RigidBody.
// - Set their mass, inertia tensor, and damping.
// - Set their initial location, orientation, velocity and rotation.
// - Apply any permanent forces (such as gravity).
//
// Then each frame:
//
// - Apply any transient forces (such as springs or thrusts).
// - Call eulerIntegrate on each body in turn.
// - Fill an array of Contact instances with all contacts on all bodies.
// - Call ContactResolver::resolveContacts to resolve the contacts.
// - Call calculateInternals to update the bodies' internal properties (such as the transform matrix).
// - Render the bodies.
//
// -- Legal
//
// This documentation is distributed under license. Use of this documentation implies agreement with all terms and conditions of the accompanying software and documentation license.
#include "precision.h"

#include <math.h>

#ifndef CYCLONE_CORE_H
#define CYCLONE_CORE_H

// The cyclone namespace includes all cyclone functions and classes. It is defined as a namespace to allow function and class names to be simple without causing conflicts.
namespace cyclone {

	// Holds the value for energy under which a body will be put to sleep. This is a global value for the whole solution. 
	// By default it is 0.1, which is fine for simulation when gravity is about 20 units per second squared, masses are about one, and other forces are around that of gravity. 
	// It may need tweaking if your simulation is drastically different to this.
	extern				double			sleepEpsilon;

	// Sets the current sleep epsilon value to use from this point on: the kinetic energy under which a body may be put to sleep. Bodies are put to sleep if they appear to have a stable kinetic energy less than this value. 
	// For simulations that often have low values (such as slow moving, or light objects), this may need reducing.
	// The value is global; all bodies will use it.
						void			setSleepEpsilon			(double value);
						double			getSleepEpsilon			();	// Gets the current value of the sleep epsilon parameter. Returns the current value of the parameter.

	// Holds a vector in 3 dimensions. Four data members are allocated to ensure alignment in an array.
	// This class contains a lot of inline methods for basic mathematics. The implementations are included in the header file.
	struct Vector3 {
							double			x						;	// Holds the value along the x axis. 
							double			y						;	// Holds the value along the y axis. 
							double			z						;	// Holds the value along the z axis. 

		static				const Vector3	GRAVITY;
		static				const Vector3	HIGH_GRAVITY;
		static				const Vector3	UP;
		static				const Vector3	RIGHT;
		static				const Vector3	OUT_OF_SCREEN;
		static				const Vector3	X;
		static				const Vector3	Y;
		static				const Vector3	Z;

		inline constexpr					Vector3					(double _x = 0, double _y = 0, double _z = 0)						: x(_x), y(_y), z(_z)														{}

		inline constexpr	bool			operator==				(const Vector3& other)							const	noexcept	{ return x == other.x && y == other.y && z == other.z;						}	// Checks if the two vectors have identical components.
		inline constexpr	bool			operator!=				(const Vector3& other)							const	noexcept	{ return !(*this == other);													}	// Checks if the two vectors have non-identical components.

							const double&	operator[]				(uint32_t i)									const				{ if(2 < i) throw("Invalid vector element."); return (&x)[i];				}
							double&			operator[]				(uint32_t i)														{ if(2 < i) throw("Invalid vector element."); return (&x)[i];				}

							Vector3			operator+				(const Vector3& v)								const	noexcept	{ return {x + v.x, y + v.y, z + v.z};										}	// Returns the value of the given vector added to this.
							Vector3			operator-				(const Vector3& v)								const	noexcept	{ return {x - v.x, y - v.y, z - v.z};										}	// Returns the value of the given vector subtracted from this.
							Vector3			operator*				(const double value)							const	noexcept	{ return {x * value, y * value, z * value};									}	// Returns a copy of this vector scaled the given value.
							double			operator*				(const Vector3 &vector)							const	noexcept	{ return x * vector.x + y * vector.y + z * vector.z;						}	// Calculates and returns the scalar product of this vector with the given vector.
							bool			operator<				(const Vector3& other)							const	noexcept	{ return x  < other.x && y  < other.y && z  < other.z;						}	// Checks if this vector is component-by-component less than the other. This does not behave like a single-value comparison: !(a < b) does not imply (b >= a).
							bool			operator>				(const Vector3& other)							const	noexcept	{ return x  > other.x && y  > other.y && z  > other.z;						}	// Checks if this vector is component-by-component less than the other. This does not behave like a single-value comparison: !(a < b) does not imply (b >= a).
							bool			operator<=				(const Vector3& other)							const	noexcept	{ return x <= other.x && y <= other.y && z <= other.z;						}	// Checks if this vector is component-by-component less than the other. This does not behave like a single-value comparison: !(a <= b) does not imply (b > a).
							bool			operator>=				(const Vector3& other)							const	noexcept	{ return x >= other.x && y >= other.y && z >= other.z;						}	// Checks if this vector is component-by-component less than the other. This does not behave like a single-value comparison: !(a <= b) does not imply (b > a).
							void			operator+=				(const Vector3& v)										noexcept	{ x +=   v.x; y +=   v.y; z +=   v.z;										}	// Adds the given vector to this. 
							void			operator-=				(const Vector3& v)										noexcept	{ x -=   v.x; y -=   v.y; z -=   v.z;										}	// Subtracts the given vector from this. 
							void			operator*=				(const double value)									noexcept	{ x *= value; y *= value; z *= value;										}	// Multiplies this vector by the given scalar.
							inline void		operator%=				(const Vector3 &vector)									noexcept	{ *this = vectorProduct(vector);											}	// Updates this vector to be the vector product of its current value and the given vector.
							inline Vector3	operator%				(const Vector3 &vector)							const	noexcept	{ return vectorProduct(vector);												}

 							void			clear					()														noexcept	{ x = y = z = 0;															}	// Zero all the components of the vector.
							void			invert					()														noexcept	{ x = -x; y = -y; z = -z;													}	// Flips all the components of the vector.
							double			squareMagnitude			()												const	noexcept	{ return x * x + y * y + z * z;												}	// Gets the squared magnitude of this vector.
							inline double	magnitude				()												const	noexcept	{ double sqLen = squareMagnitude(); return sqLen ? real_sqrt(sqLen) : 0;	}	// Gets the magnitude of this vector.
							double			scalarProduct			(const Vector3 &vector)							const	noexcept	{ return x * vector.x + y * vector.y + z * vector.z;						}	// Calculates and returns the scalar product of this vector with the given vector.
							Vector3			componentProduct		(const Vector3 &vector)							const	noexcept	{ return {x * vector.x, y * vector.y, z * vector.z};						}	// Calculates and returns a component-wise product of this vector with the given vector.
							void			componentProductUpdate	(const Vector3 &vector)									noexcept	{ x *= vector.x; y *= vector.y; z *= vector.z;								}	// Performs a component-wise product with the given vector and sets this vector to its result.
							Vector3			vectorProduct			(const Vector3 &vector)							const	noexcept	{	// Calculates and returns the vector product of this vector with the given vector.
			return 
				{	y * vector.z - z * vector.y
				,	z * vector.x - x * vector.z
				,	x * vector.y - y * vector.x
				};	
		}	
		// Adds the given vector to this, scaled by the given amount.
							void			addScaledVector			(const Vector3& vector, double scale)					noexcept	{
			x									+= vector.x * scale;
			y									+= vector.y * scale;
			z									+= vector.z * scale;
		}
		// Limits the size of the vector to the given maximum.
							void			trim					(double size)														{
			if (squareMagnitude() > size*size) {
				normalise();
				x									*= size;
				y									*= size;
				z									*= size;
			}
		}
		// Turns a non-zero vector into a vector of unit length.
							void			normalise				()																	{
			double									l						= magnitude();
			if(l > 0)
				(*this)								*= 1.0 / l;
		}
		// Returns the normalised version of a vector.
							Vector3			unit					()												const				{
			Vector3									result					= *this;
			result.normalise();
			return result;
		}
	};

	// Holds a three degree of freedom orientation.
	// Quaternions have several mathematical properties that make them useful for representing orientations, but require four items of data to hold the three degrees of freedom. 
	// These four items of data can be viewed as the coefficients of a complex number with three imaginary parts. The mathematics of the quaternion is then defined and is roughly correspondent to the math of 3D rotations. 
	// A quaternion is only a valid rotation if it is normalised: i.e. it has a length of 1.
	// Note: Angular velocity and acceleration can be correctly represented as vectors. Quaternions are only needed for orientation.
	// The explicit constructor creates a quaternion with the given components.
	// r The real component of the rigid body's orientation quaternion.
	// i The first complex component of the rigid body's orientation quaternion.
	// j The second complex component of the rigid body's orientation quaternion.
	// k The third complex component of the rigid body's orientation quaternion.
	// The given orientation does not need to be normalised, and can be zero. This function will not alter the given values, or normalise the quaternion. To normalise the quaternion (and make a zero quaternion a legal rotation), use the normalise function.
	struct Quaternion {
		double							r;	// Holds the real component of the quaternion.
		double							i;	// Holds the first complex component of the quaternion.
		double							j;	// Holds the second complex component of the quaternion.
		double							k;	// Holds the third complex component of the quaternion.
		// The default constructor creates a quaternion representing a zero rotation.
										Quaternion						(const double _r = 1, const double _i = 0, const double _j = 0, const double _k = 0) : r(_r), i(_i), j(_j), k(_k)	{}
		// Normalises the quaternion to unit length, making it a valid orientation quaternion.
		void							normalise						()																													{
			double								d								= r*r+i*i+j*j+k*k;
			
			// Check for zero length quaternion, and use the no-rotation
			// quaternion in that case.
			if (d < real_epsilon) {
				r								= 1;
				return;
			}
			
			d								= ((double)1.0)/real_sqrt(d);
			r								*= d;
			i								*= d;
			j								*= d;
			k								*= d;
		}
		
		// Multiplies the quaternion by the given quaternion. multiplier: The quaternion by which to multiply.
		void							operator*=						(const Quaternion &multiplier)																						{
			Quaternion							q								= *this;
			r								= q.r * multiplier.r - q.i * multiplier.i - q.j * multiplier.j - q.k * multiplier.k;
			i								= q.r * multiplier.i + q.i * multiplier.r + q.j * multiplier.k - q.k * multiplier.j;
			j								= q.r * multiplier.j + q.j * multiplier.r + q.k * multiplier.i - q.i * multiplier.k;
			k								= q.r * multiplier.k + q.k * multiplier.r + q.i * multiplier.j - q.j * multiplier.i;
		}
		
		// Adds the given vector to this, scaled by the given amount. This is used to update the orientation quaternion by a rotation and time.
		// vector	: The vector to add.
		// scale	: The amount of the vector to add.
		void							addScaledVector					(const Vector3& vector, double scale)																				{
			Quaternion							q								= {0, vector.x * scale, vector.y * scale, vector.z * scale};
			q								*= *this;
			r								+= q.r * ((double)0.5);
			i								+= q.i * ((double)0.5);
			j								+= q.j * ((double)0.5);
			k								+= q.k * ((double)0.5);
		}
		
		void							rotateByVector					(const Vector3& vector)																								{
			Quaternion							q								= {0, vector.x, vector.y, vector.z};
			(*this)							*= q;
		}
	};

	// Holds a transform matrix, consisting of a rotation matrix and a position. The matrix has 12 elements, it is assumed that the remaining four are (0,0,0,1); producing a homogenous matrix.
	struct Matrix4 {
		double			data[12];						// Holds the transform matrix data in array form.
		// Creates an identity matrix.
						Matrix4							()																													{
			data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0;
			data[0] = data[5] = data[10] = 1;
		}
		
		// Sets the matrix to be a diagonal matrix with the given coefficients.
		void			setDiagonal						(double a, double b, double c)																						{
			data[0]			= a;
			data[5]			= b;
			data[10]		= c;
		}
		
		// Returns a matrix which is this matrix multiplied by the given other matrix.
		Matrix4			operator*						(const Matrix4 &o)																							const	{
			Matrix4				result;
			result.data[0]	= (o.data[0]*data[0]) + (o.data[4]*data[1]) + (o.data[8]*data[2]);
			result.data[4]	= (o.data[0]*data[4]) + (o.data[4]*data[5]) + (o.data[8]*data[6]);
			result.data[8]	= (o.data[0]*data[8]) + (o.data[4]*data[9]) + (o.data[8]*data[10]);

			result.data[1]	= (o.data[1]*data[0]) + (o.data[5]*data[1]) + (o.data[9]*data[2]);
			result.data[5]	= (o.data[1]*data[4]) + (o.data[5]*data[5]) + (o.data[9]*data[6]);
			result.data[9]	= (o.data[1]*data[8]) + (o.data[5]*data[9]) + (o.data[9]*data[10]);

			result.data[2]	= (o.data[2]*data[0]) + (o.data[6]*data[1]) + (o.data[10]*data[2]);
			result.data[6]	= (o.data[2]*data[4]) + (o.data[6]*data[5]) + (o.data[10]*data[6]);
			result.data[10]	= (o.data[2]*data[8]) + (o.data[6]*data[9]) + (o.data[10]*data[10]);

			result.data[3]	= (o.data[3]*data[0]) + (o.data[7]*data[1]) + (o.data[11]*data[2]) + data[3];
			result.data[7]	= (o.data[3]*data[4]) + (o.data[7]*data[5]) + (o.data[11]*data[6]) + data[7];
			result.data[11]	= (o.data[3]*data[8]) + (o.data[7]*data[9]) + (o.data[11]*data[10]) + data[11];

			return result;
		}

	    // Transform the given vector by this matrix.
		Vector3			operator*					(const Vector3 &vector)																								const	{
			return 
				{	vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3]
				,	vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7]
				,	vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]
				};
		}

		void			setInverse					(const Matrix4 &matrixToInvert);		// Sets the matrix to be the inverse of the given matrix. matrixToInvert: The matrix to invert and use to set this.
		void			invert						()																															{ setInverse(*this);		}
		Vector3			transform					(const Vector3 &vector)																								const	{ return (*this) * vector;	}	// Transform the given vector by this matrix.
		double			getDeterminant				()																													const;
		Matrix4			inverse						()																													const	{ // Returns a new matrix containing the inverse of this matrix. 
			Matrix4 result;
			result.setInverse(*this);
			return result;
		}

		// Transform the given direction vector by this matrix. When a direction is converted between frames of reference, there is no translation required.
		Vector3			transformDirection			(const Vector3 &vector)																								const	{
			return 
				{	vector.x * data[0] + vector.y * data[1] + vector.z * data[2]
				,	vector.x * data[4] + vector.y * data[5] + vector.z * data[6]
				,	vector.x * data[8] + vector.y * data[9] + vector.z * data[10]
				};
		}

		// Transform the given direction vector by the transformational inverse of this matrix.
		// This function relies on the fact that the inverse of a pure rotation matrix is its transpose. It separates the translational and rotation components, transposes the rotation, and multiplies out. If the matrix is not a scale and shear free transform matrix, then this function will not give correct results.
		// When a direction is converted between frames of reference, there is no translation required.
		Vector3			transformInverseDirection	(const Vector3 &vector)																								const	{
			return 
				{	vector.x * data[0] + vector.y * data[4] + vector.z * data[8]
				,	vector.x * data[1] + vector.y * data[5] + vector.z * data[9]
				,	vector.x * data[2] + vector.y * data[6] + vector.z * data[10]
				};
		}

		// Transform the given vector by the transformational inverse of this matrix.
		// This function relies on the fact that the inverse of a pure rotation matrix is its transpose. It separates the translational and rotation components, transposes the rotation, and multiplies out. 
		// If the matrix is not a scale and shear free transform matrix, then this function will not give correct results.
		Vector3		transformInverse			(const Vector3 &vector)																									const	{
			Vector3						tmp				= vector;
			tmp.x					-= data[3];
			tmp.y					-= data[7];
			tmp.z					-= data[11];
			return 
				{	tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8]
				,	tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9]
				,	tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]
				};
		}
		// Gets a vector representing one axis (i.e. one column) in the matrix. i: The row to return. Row 3 corresponds to the position of the transform matrix.
		Vector3		getAxisVector				(int i)																													const	{ return {data[i], data[i+4], data[i+8]}; }	
		// Sets this matrix to be the rotation matrix corresponding to the given quaternion.
		void		setOrientationAndPos		(const Quaternion &q, const Vector3 &pos)																						{
		    data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
		    data[1] = 2*q.i*q.j + 2*q.k*q.r;
		    data[2] = 2*q.i*q.k - 2*q.j*q.r;
		    data[3] = pos.x;

		    data[4] = 2*q.i*q.j - 2*q.k*q.r;
		    data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
		    data[6] = 2*q.j*q.k + 2*q.i*q.r;
		    data[7] = pos.y;

		    data[8] = 2*q.i*q.k + 2*q.j*q.r;
		    data[9] = 2*q.j*q.k - 2*q.i*q.r;
		    data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
		    data[11] = pos.z;
		}

		// Fills the given array with this transform matrix, so it is usable as an open-gl transform matrix. OpenGL uses a column major format, so that the values are transposed as they are written.
		void fillGLArray(float array[16]) const {
		    array[0] = (float)data[0];
		    array[1] = (float)data[4];
		    array[2] = (float)data[8];
		    array[3] = (float)0;

		    array[4] = (float)data[1];
		    array[5] = (float)data[5];
		    array[6] = (float)data[9];
		    array[7] = (float)0;

		    array[8] = (float)data[2];
		    array[9] = (float)data[6];
		    array[10] = (float)data[10];
		    array[11] = (float)0;

		    array[12] = (float)data[3];
		    array[13] = (float)data[7];
		    array[14] = (float)data[11];
		    array[15] = (float)1;
		}
	};

	// Holds an inertia tensor, consisting of a 3x3 row-major matrix. This matrix is not padding to produce an aligned structure, since it is most commonly used with a mass (single real) and two damping coefficients to make the 12-element characteristics array of a rigid body.
	struct Matrix3 {
		double		data		[9]			= {};					// Holds the tensor matrix data in array form.

		constexpr	Matrix3					()																												= default;

		// Creates a new matrix with the given three vectors making up its columns.
					Matrix3					(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree)										{ setComponents(compOne, compTwo, compThree); }
		// Creates a new matrix with explicit coefficients.
					Matrix3					(double c0, double c1, double c2, double c3, double c4, double c5, double c6, double c7, double c8)				{
			data[0] = c0; data[1] = c1; data[2] = c2;
			data[3] = c3; data[4] = c4; data[5] = c5;
			data[6] = c6; data[7] = c7; data[8] = c8;
		}

		// Sets the matrix to be a diagonal matrix with the given values along the leading diagonal.
		void		setDiagonal				(double a, double b, double c)																					{ setInertiaTensorCoeffs(a, b, c); }

		// Sets the value of the matrix from inertia tensor values.
		void		setInertiaTensorCoeffs	(double ix, double iy, double iz, double ixy=0, double ixz=0, double iyz=0)										{
		    data[0]		= ix;
		    data[1]		= data[3] = -ixy;
		    data[2]		= data[6] = -ixz;
		    data[4]		= iy;
		    data[5]		= data[7] = -iyz;
		    data[8]		= iz;
		}

		// Sets the value of the matrix as an inertia tensor of a rectangular block aligned with the body's coordinate system with the given axis half-sizes and mass.
		void		setBlockInertiaTensor	(const Vector3 &halfSizes, double mass)																			{
		    Vector3			squares					= halfSizes.componentProduct(halfSizes);
		    setInertiaTensorCoeffs(0.3f*mass*(squares.y + squares.z),
		        0.3f*mass*(squares.x + squares.z),
		        0.3f*mass*(squares.x + squares.y));
		}

		// Sets the matrix to be a skew symmetric matrix based on the given vector. The skew symmetric matrix is the equivalent of the vector product. So if a,b are vectors. a x b = A_s b where A_s is the skew symmetric form of a.
		void		setSkewSymmetric		(const Vector3 vector)																							{
		    data[0]		= data[4] = data[8] = 0;
		    data[1]		= -vector.z;
		    data[2]		= vector.y;
		    data[3]		= vector.z;
		    data[5]		= -vector.x;
		    data[6]		= -vector.y;
		    data[7]		= vector.x;
		}

		// Sets the matrix values from the given three vector components. These are arranged as the three columns of the vector.
		void		setComponents			(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree)										{
		    data[0]		= compOne.x;
		    data[1]		= compTwo.x;
		    data[2]		= compThree.x;
		    data[3]		= compOne.y;
		    data[4]		= compTwo.y;
		    data[5]		= compThree.y;
		    data[6]		= compOne.z;
		    data[7]		= compTwo.z;
		    data[8]		= compThree.z;

		}

		// Transform the given vector by this matrix.
		Vector3		operator*				(const Vector3 &vector)																		const				{
		    return 
				{ vector.x * data[0] + vector.y * data[1] + vector.z * data[2]
				, vector.x * data[3] + vector.y * data[4] + vector.z * data[5]
				, vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
				};
		}


		Vector3		getRowVector			(int i)																						const				{ return {data[i*3]	, data[i*3+1]	, data[i*3+2]};		}	// Gets a vector representing one row in the matrix.
		Vector3		getAxisVector			(int i)																						const				{ return {data[i]	, data[i+3]		, data[i+6]};		}	// Gets a vector representing one axis (i.e. one column) in the matrix.
		Vector3		transform				(const Vector3 &vector)																		const				{ return (*this) * vector; }	// Transform the given vector by this matrix.
		Vector3		transformTranspose		(const Vector3 &vector)																		const				{	// Transform the given vector by the transpose of this matrix.
		    return 
				{	vector.x * data[0] + vector.y * data[3] + vector.z * data[6]
				,	vector.x * data[1] + vector.y * data[4] + vector.z * data[7]
				,	vector.x * data[2] + vector.y * data[5] + vector.z * data[8]
				};
		}

		// Sets the matrix to be the inverse of the given matrix.
		void		setInverse				(const Matrix3 &m)																								{
			double t4	= m.data[0] * m.data[4];
			double t6	= m.data[0] * m.data[5];
			double t8	= m.data[1] * m.data[3];
			double t10	= m.data[2] * m.data[3];
			double t12	= m.data[1] * m.data[6];
			double t14	= m.data[2] * m.data[6];

			// Calculate the determinant
			double t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8]+
			            t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

			// Make sure the determinant is non-zero.
			if (t16 == (double)0.0f) 
				return;
			double t17 = 1/t16;

			data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*t17;
			data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*t17;
			data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*t17;
			data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*t17;
			data[4] = (m.data[0]*m.data[8]-t14)*t17;
			data[5] = -(t6-t10)*t17;
			data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*t17;
			data[7] = -(m.data[0]*m.data[7]-t12)*t17;
			data[8] = (t4-t8)*t17;
		}

		// Returns a new matrix containing the inverse of this matrix.
		Matrix3 inverse() const
		{
		    Matrix3 result;
		    result.setInverse(*this);
		    return result;
		}

		// Inverts the matrix.
		void invert() { setInverse(*this); }

		// Sets the matrix to be the transpose of the given matrix.
		void setTranspose(const Matrix3 &m) {
		    data[0] = m.data[0];
		    data[1] = m.data[3];
		    data[2] = m.data[6];
		    data[3] = m.data[1];
		    data[4] = m.data[4];
		    data[5] = m.data[7];
		    data[6] = m.data[2];
		    data[7] = m.data[5];
		    data[8] = m.data[8];
		}

		// Returns a new matrix containing the transpose of this matrix.
		Matrix3 transpose() const {
		    Matrix3 result;
		    result.setTranspose(*this);
		    return result;
		}

		// Returns a matrix which is this matrix multiplied by the given other matrix.
		Matrix3 operator*(const Matrix3 &o) const {
		    return Matrix3(
		        data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
		        data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
		        data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

		        data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
		        data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
		        data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

		        data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
		        data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
		        data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
		        );
		}

		// Multiplies this matrix in place by the given other matrix.
		void operator*=(const Matrix3 &o)
		{
		    double t1;
		    double t2;
		    double t3;

		    t1 = data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6];
		    t2 = data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7];
		    t3 = data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8];
		    data[0] = t1;
		    data[1] = t2;
		    data[2] = t3;

		    t1 = data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6];
		    t2 = data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7];
		    t3 = data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8];
		    data[3] = t1;
		    data[4] = t2;
		    data[5] = t3;

		    t1 = data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6];
		    t2 = data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7];
		    t3 = data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8];
		    data[6] = t1;
		    data[7] = t2;
		    data[8] = t3;
		}

		// Multiplies this matrix in place by the given scalar.
		void operator*=(const double scalar) {
		    data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
		    data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
		    data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
		}

		// Does a component-wise addition of this matrix and the given matrix.
		void operator+=(const Matrix3 &o) {
		    data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
		    data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
		    data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
		}

		// Sets this matrix to be the rotation matrix corresponding to the given quaternion.
		void setOrientation(const Quaternion &q) {
		    data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
		    data[1] = 2*q.i*q.j + 2*q.k*q.r;
		    data[2] = 2*q.i*q.k - 2*q.j*q.r;
		    data[3] = 2*q.i*q.j - 2*q.k*q.r;
		    data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
		    data[5] = 2*q.j*q.k + 2*q.i*q.r;
		    data[6] = 2*q.i*q.k + 2*q.j*q.r;
		    data[7] = 2*q.j*q.k - 2*q.i*q.r;
		    data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
		}

		static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, double prop);	// Interpolates a couple of matrices.
    };
}

#endif // CYCLONE_CORE_H
