// Copyright (c) Icosagon 2003. All Rights Reserved.

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "core.h"

namespace cyclone {

	struct RigidBody {
        real								InverseMass;
        Matrix3								InverseInertiaTensor;
        real								LinearDamping;
        real								AngularDamping;
        Vector3								Position;
        Quaternion							Orientation;
        Vector3								Velocity;
        Vector3								Rotation;
        Matrix3								InverseInertiaTensorWorld;
        real								Motion;
        bool								IsAwake;
        bool								CanSleep;
        Matrix4								TransformMatrix;
        Vector3								ForceAccum;
        Vector3								TorqueAccum;
        Vector3								Acceleration;
        Vector3								LastFrameAcceleration;

        void								calculateDerivedData();
        void								integrate(real duration);
        void								setMass(const real mass);
        real								getMass() const;
        void								setInverseMass(const real inverseMass);
        real								getInverseMass() const;
        bool								hasFiniteMass() const;
        void								setInertiaTensor(const Matrix3 &inertiaTensor);
        void								getInertiaTensor(Matrix3 *inertiaTensor) const;
        Matrix3								getInertiaTensor() const;
        void								getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
        Matrix3								getInertiaTensorWorld() const;
        void								setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
        void								getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;
        Matrix3								getInverseInertiaTensor() const;
        void								getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
        Matrix3								getInverseInertiaTensorWorld() const;
        void								setDamping(const real linearDamping, const real angularDamping);

		void								setPosition(const real x, const real y, const real z);
        void								getPosition(Vector3 *position) const;
		void								setOrientation(const Quaternion &orientation);
        void								setOrientation(const real r, const real i, const real j, const real k);
        void								getOrientation(Quaternion *orientation) const;

		void								getOrientation(Matrix3 *matrix) const;
        void								getOrientation(real matrix[9]) const;
        void								getTransform(Matrix4 *transform) const;
        void								getTransform(real matrix[16]) const;
        void								getGLTransform(float matrix[16]) const;
        Matrix4								getTransform() const;
        Vector3								GetPointInLocalSpace(const Vector3 &point) const;
        Vector3								getPointInWorldSpace(const Vector3 &point) const;
        Vector3								getDirectionInLocalSpace(const Vector3 &direction) const;
        Vector3								getDirectionInWorldSpace(const Vector3 &direction) const;
        void								setVelocity(const real x, const real y, const real z);
        void								getVelocity(Vector3 *velocity) const;
		void								addVelocity(const Vector3 &deltaVelocity);
        void								setRotation(const real x, const real y, const real z);
        void								getRotation(Vector3 *rotation) const;

		void								addRotation(const Vector3 &deltaRotation);
        void								setAwake(const bool awake=true);
        void								setCanSleep(const bool canSleep=true);
        void								getLastFrameAcceleration(Vector3 *linearAcceleration) const;
        Vector3								getLastFrameAcceleration() const;
        void								clearAccumulators();
        void								addForce(const Vector3 &force);
        void								addForceAtPoint(const Vector3 &force, const Vector3 &point);
        void								addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void								addTorque(const Vector3 &torque);
        void								setAcceleration(const real x, const real y, const real z);
        void								getAcceleration(Vector3 *acceleration) const;
    };

} // namespace cyclone

#endif // CYCLONE_BODY_H
