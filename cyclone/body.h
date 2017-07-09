// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

namespace cyclone {
#pragma pack(push, 1)
	// Internal function that checks the validity of an inverse inertia tensor.
	static inline constexpr	void		checkInverseInertiaTensor		(const Matrix3 &/*iitWorld*/)											noexcept	{}	// TODO: Perform a validity check in an assert.

	struct SMass3D {
				double						InverseMass;
				Matrix3						InverseInertiaTensor;
				double						LinearDamping;
				double						AngularDamping;

		inline	void						setInertiaTensor				(const Matrix3 &inertiaTensor)													{ InverseInertiaTensor.setInverse(inertiaTensor); checkInverseInertiaTensor(InverseInertiaTensor);	}
		inline	void						setInverseInertiaTensor			(const Matrix3 &inverseInertiaTensor)											{ checkInverseInertiaTensor(inverseInertiaTensor); InverseInertiaTensor = inverseInertiaTensor;		}
		inline	double						getMass							()																	const		{ return (InverseMass == 0) ? REAL_MAX : 1.0 / InverseMass;											}
		inline	bool						hasFiniteMass					()																	const		{ return InverseMass >= 0.0f;																		}
		inline	void						setMass							(const double mass)																{ InverseMass = 1.0 / mass;																			}
		inline	void						setDamping						(const double linearDamping, const double angularDamping)						{	
			LinearDamping						= linearDamping;
			AngularDamping						= angularDamping;
		}
	};

	struct SForce3D {
				Vector3						Velocity;
				Vector3						Rotation;
				Vector3						Acceleration;
	};

	struct SPivot3D {
				Vector3						Position;
				Quaternion					Orientation;
		inline	void						setOrientation					(const Quaternion &orientation)													{ Orientation = orientation;	Orientation.normalise();			}
		inline	void						setOrientation					(const double r, const double i, const double j, const double k)				{ Orientation = {r, i, j, k};	Orientation.normalise();			}
	};

	struct RigidBody {
				SMass3D						Mass;
				SForce3D					Force;
				SPivot3D					Pivot;
				double						Motion;
				bool						IsAwake;
				bool						CanSleep;
				Matrix4						TransformMatrix;
				Matrix3						InverseInertiaTensorWorld;
				Vector3						AccumulatedForce;
				Vector3						AccumulatedTorque;
				Vector3						LastFrameAcceleration;
				
				void						CalculateDerivedData			();
				void						integrate						(double duration);

				void						getGLTransform					(float matrix[16])													const;
				void						setAwake						(const bool awake = true);
				void						setCanSleep						(const bool canSleep = true);
				void						addForceAtPoint					(const Vector3 &force, const Vector3 &point);

				void						getOrientation					(double matrix[9])													const;
		inline	void						getOrientation					(Matrix3 *matrix)													const		{ getOrientation(matrix->data);										}
		inline	Vector3						GetPointInLocalSpace			(const Vector3 &point)												const		{ return TransformMatrix.transformInverse			(point);		}
		inline	Vector3						getPointInWorldSpace			(const Vector3 &point)												const		{ return TransformMatrix.transform					(point);		}
		inline	Vector3						getDirectionInLocalSpace		(const Vector3 &direction)											const		{ return TransformMatrix.transformInverseDirection	(direction);	}
		inline	Vector3						getDirectionInWorldSpace		(const Vector3 &direction)											const		{ return TransformMatrix.transformDirection			(direction);	}
		inline	void						addForceAtBodyPoint				(const Vector3 &force, const Vector3 &point)									{ addForceAtPoint(force, getPointInWorldSpace(point));				}	// Convert to coordinates relative to center of mass.
		inline	void						addTorque						(const Vector3 &torque)															{ AccumulatedTorque	+= torque	; IsAwake = true;					}
		inline	void						addForce						(const Vector3 &force)															{ AccumulatedForce	+= force	; IsAwake = true;					}
		inline	void						clearAccumulators				()																				{
			AccumulatedForce	.clear();
			AccumulatedTorque	.clear();
		}
	};
#pragma pack(pop)

} // namespace cyclone

#endif // CYCLONE_BODY_H
