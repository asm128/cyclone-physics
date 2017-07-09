// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "core.h"

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

namespace cyclone {

	struct RigidBody {
		double								InverseMass;
		Matrix3								InverseInertiaTensor;
		double								LinearDamping;
		double								AngularDamping;
		Vector3								Position;
		Quaternion							Orientation;
		Vector3								Velocity;
		Vector3								Rotation;
		Matrix3								InverseInertiaTensorWorld;
		double								Motion;
		bool								IsAwake;
		bool								CanSleep;
		Matrix4								TransformMatrix;
		Vector3								AccumulatedForce;
		Vector3								TorqueAccum;
		Vector3								Acceleration;
		Vector3								LastFrameAcceleration;
		
		void								calculateDerivedData();
		void								integrate(double duration);
		void								setMass(const double mass);
		double								getMass() const;
		bool								hasFiniteMass() const;
		void								setInertiaTensor(const Matrix3 &inertiaTensor);
		void								getInertiaTensor(Matrix3 *inertiaTensor) const;
		Matrix3								getInertiaTensor() const;
		void								getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
		Matrix3								getInertiaTensorWorld() const;
		void								setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
		void								setDamping(const double linearDamping, const double angularDamping);
		
		void								getPosition(Vector3 *position) const;
		void								setOrientation(const Quaternion &orientation);
		void								setOrientation(const double r, const double i, const double j, const double k);
		
		void								getOrientation(Matrix3 *matrix) const;
		void								getOrientation(double matrix[9]) const;
		void								getTransform(Matrix4 *transform) const;
		void								getTransform(double matrix[16]) const;
		void								getGLTransform(float matrix[16]) const;
		Matrix4								getTransform() const;
		Vector3								GetPointInLocalSpace(const Vector3 &point) const;
		Vector3								getPointInWorldSpace(const Vector3 &point) const;
		Vector3								getDirectionInLocalSpace(const Vector3 &direction) const;
		Vector3								getDirectionInWorldSpace(const Vector3 &direction) const;
		void								setAwake(const bool awake=true);
		void								setCanSleep(const bool canSleep=true);
		void								clearAccumulators();
		void								addForce(const Vector3 &force);
		void								addForceAtPoint(const Vector3 &force, const Vector3 &point);
		void								addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
		void								addTorque(const Vector3 &torque);
	};

} // namespace cyclone

#endif // CYCLONE_BODY_H
