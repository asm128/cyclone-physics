// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

namespace cyclone {
	struct RigidBody {
				double						InverseMass;
				Matrix3						InverseInertiaTensor;
				double						LinearDamping;
				double						AngularDamping;
				Vector3						Position;
				Quaternion					Orientation;
				Vector3						Velocity;
				Vector3						Rotation;
				Matrix3						InverseInertiaTensorWorld;
				double						Motion;
				bool						IsAwake;
				bool						CanSleep;
				Matrix4						TransformMatrix;
				Vector3						AccumulatedForce;
				Vector3						AccumulatedTorque;
				Vector3						Acceleration;
				Vector3						LastFrameAcceleration;
				
				void						calculateDerivedData			();
				void						integrate						(double duration);

		inline	double						getMass							()																	const	{ return (InverseMass == 0) ? REAL_MAX : 1.0 / InverseMass;			}
		inline	Matrix3						getInertiaTensorWorld			()																	const	{ Matrix3 it; getInertiaTensorWorld	(&it); return it;				}
		inline	Matrix3						getInertiaTensor				()																	const	{ Matrix3 it; getInertiaTensor		(&it); return it;				}
		inline	void						getInertiaTensorWorld			(Matrix3 *inertiaTensor)											const	{ inertiaTensor->setInverse(InverseInertiaTensorWorld);				}
		inline	void						getInertiaTensor				(Matrix3 *inertiaTensor)											const	{ inertiaTensor->setInverse(InverseInertiaTensor);					}
		inline	bool						hasFiniteMass					()																	const	{ return InverseMass >= 0.0f;										}
		inline	void						setMass							(const double mass)															{ InverseMass = 1.0 / mass;											}
				void						setInertiaTensor				(const Matrix3 &inertiaTensor);
				void						setInverseInertiaTensor			(const Matrix3 &inverseInertiaTensor);
		inline	void						setDamping						(const double linearDamping, const double angularDamping)					{	
			LinearDamping						= linearDamping;
			AngularDamping						= angularDamping;
		}

				void						getOrientation					(double matrix[9])													const;
		inline	void						getOrientation					(Matrix3 *matrix)													const	{ getOrientation(matrix->data);										}
		inline	void						setOrientation					(const Quaternion &orientation)												{ Orientation = orientation;	Orientation.normalise();			}
		inline	void						setOrientation					(const double r, const double i, const double j, const double k)			{ Orientation = {r, i, j, k};	Orientation.normalise();			}

		inline	void						getTransform					(Matrix4 *transform)												const	{ *transform = TransformMatrix;										}
				void						getTransform					(double matrix[16])													const;
				void						getGLTransform					(float matrix[16])													const;
		inline	Vector3						GetPointInLocalSpace			(const Vector3 &point)												const	{ return TransformMatrix.transformInverse			(point);		}
		inline	Vector3						getPointInWorldSpace			(const Vector3 &point)												const	{ return TransformMatrix.transform					(point);		}
		inline	Vector3						getDirectionInLocalSpace		(const Vector3 &direction)											const	{ return TransformMatrix.transformInverseDirection	(direction);	}
		inline	Vector3						getDirectionInWorldSpace		(const Vector3 &direction)											const	{ return TransformMatrix.transformDirection			(direction);	}
				void						setAwake						(const bool awake = true);
				void						setCanSleep						(const bool canSleep = true);
				void						addForceAtPoint					(const Vector3 &force, const Vector3 &point);
		inline	void						addForceAtBodyPoint				(const Vector3 &force, const Vector3 &point)								{ addForceAtPoint(force, getPointInWorldSpace(point));				}	// Convert to coordinates relative to center of mass.
		inline	void						addTorque						(const Vector3 &torque)														{ AccumulatedTorque	+= torque	; IsAwake = true;					}
		inline	void						addForce						(const Vector3 &force)														{ AccumulatedForce	+= force	; IsAwake = true;					}
		inline	void						clearAccumulators				()																			{
			AccumulatedForce	.clear();
			AccumulatedTorque	.clear();
		}
	};
} // namespace cyclone

#endif // CYCLONE_BODY_H
