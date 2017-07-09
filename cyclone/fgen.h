// This file contains the interface and sample force generators.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "body.h"
#include "pfgen.h"

#include <vector>

#ifndef CYCLONE_FGEN_H
#define CYCLONE_FGEN_H

namespace cyclone {

    // A force generator can be asked to add a force to one or more bodies.
    class ForceGenerator {
    public:
        // Overload this in implementations of the interface to calculate and update the force applied to the given rigid body.
        virtual				void							UpdateForce					(RigidBody *body, double duration)			= 0;
    };

	// A force generator that applies a gravitational force. One instance can be used for multiple rigid bodies.
	class ForceGravity : public ForceGenerator {
							Vector3							Gravity						= {};		// Holds the acceleration due to gravity.
	public:
															ForceGravity				(const Vector3 &gravity)					: Gravity(gravity)	{}	// Creates the generator with the given acceleration. 
		virtual				void							UpdateForce					(RigidBody *body, double duration)			{						// Applies the gravitational force to the given rigid body.
			if (body->hasFiniteMass()) 	// Apply the mass-scaled force to the body
				body->addForce(Gravity * body->getMass());
		}	
	};

	// A force generator that applies a Spring force.
	class Spring : public ForceGenerator {
							Vector3							connectionPoint				= {};	// The point of connection of the spring, in local coordinates.
							Vector3							otherConnectionPoint		= {};	// The point of connection of the spring to the other object, in that object's local coordinates.
							RigidBody						* other						= 0;	// The particle at the other end of the spring.

							double							springConstant				= 0;	// Holds the sprint constant.
							double							restLength					= 0;	// Holds the rest length of the spring.

	public:
															Spring
		( const Vector3			& localConnectionPt
		, RigidBody				* other
		, const Vector3			& otherConnectionPt
		, double				springConstant
		, double				restLength
		);

		virtual				void							UpdateForce					(RigidBody *body, double duration);	// Applies the spring force to the given rigid body.
	};
	
	// A force generator that applies an aerodynamic force.
	class Aero : public ForceGenerator {
	protected:
							Matrix3							Tensor						= {};	// Holds the aerodynamic tensor for the surface in body space.
							Vector3							Position					= {};	// Holds the relative position of the aerodynamic surface in body coordinates.
							const Vector3					* Windspeed					= 0;	// Holds a pointer to a vector containing the windspeed of the environment. This is easier than managing a separate windspeed vector per generator and having to update it manually as the wind changes.
	public:
		inline constexpr									Aero						(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed)			: Tensor(tensor), Position(position), Windspeed(windspeed)		{}
		virtual				void							UpdateForce					(RigidBody *body, double duration);	// Applies the force to the given rigid body.
	protected:
							void							UpdateForceFromTensor		(RigidBody *body, double duration, const Matrix3 &tensor);	// Uses an explicit tensor matrix to update the force on the given rigid body. This is exactly the same as for UpdateForce only it takes an explicit tensor.
	};

	// A force generator with a control aerodynamic surface. This requires three inertia tensors, for the two extremes and 'resting' position of the control surface. The latter tensor is the one inherited from the base class, the two extremes are defined in this class.
	class AeroControl : public Aero {
	protected:
							Matrix3							MaxTensor					= {};	// The aerodynamic tensor for the surface, when the control is at its maximum value.
							Matrix3							MinTensor					= {};	// The aerodynamic tensor for the surface, when the control is at its minimum value.
							double							ControlSetting				= 0;	// The current position of the control for this surface. This should range between -1 (in which case the minTensor value is used), through 0 (where the base-class tensor value is used) to +1 (where the maxTensor value is used).

							Matrix3							getTensor					();	// Calculates the final aerodynamic tensor for the current control setting.
	public:
															AeroControl
			( const Matrix3 &base
			, const Matrix3 &min
			, const Matrix3 &max
			, const Vector3 &position
			, const Vector3 *windspeed
			);
		inline				void							setControl					(double value)						{ ControlSetting = value; }	// Sets the control position of this control. This should range between -1 (in which case the minTensor value is used), through 0 (where the base-class tensor value is used) to +1 (where the maxTensor value is used). Values outside that range give undefined results.
							void							UpdateForce					(RigidBody *body, double duration)	{ Aero::UpdateForceFromTensor(body, duration, getTensor()); }
	};

	// A force generator with an aerodynamic surface that can be re-oriented relative to its rigid body. This derives the
	class AngledAero : public Aero {
							Quaternion						orientation					= {};	// Holds the orientation of the aerodynamic surface relative to the rigid body to which it is attached.

    public:
															AngledAero					(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);
							void							setOrientation				(const Quaternion &quat);	// Sets the relative orientation of the aerodynamic surface, relative to the rigid body it is attached to. Note that this doesn't affect the point of connection of the surface to the body.

		virtual				void							UpdateForce					(RigidBody *body, double duration);	// Applies the force to the given rigid body.
	};

	// A force generator to apply a buoyant force to a rigid body.
	class Buoyancy : public ForceGenerator {
							Vector3							centreOfBuoyancy			= {};		// The centre of buoyancy of the rigid body, in body coordinates.
							double							maxDepth					= 0;		// The maximum submersion depth of the object before it generates its maximum buoyancy force.
							double							volume						= 0;		// The volume of the object.
							double							waterHeight					= 0;		// The height of the water plane above y=0. The plane will be parallel to the XZ plane.
							double							liquidDensity				= 1000.0f;	// The density of the liquid. Pure water has a density of 1000kg per cubic meter.
	public:
															Buoyancy
			(	const Vector3	& centreOfBuoyancy
			,   double			maxDepth
			,	double			volume
			,	double			waterHeight
			,	double			liquidDensity	= 1000.0f
			);

		virtual				void							UpdateForce					(RigidBody *body, double duration);	// Applies the force to the given rigid body.
	};

    // Holds all the force generators and the bodies they apply to.
	class ForceRegistry {
	protected:
		// Keeps track of one force generator and the body it applies to.
		struct ForceRegistration {
			RigidBody											* body						= {};
			ForceGenerator										* fg						= {};
		};
		// Holds the list of registrations.
		typedef				std::vector<ForceRegistration>	Registry;
							Registry						registrations;

	public:
							void							add							(RigidBody* body, ForceGenerator *fg);	// Registers the given force generator to apply to the given body.
							void							remove						(RigidBody* body, ForceGenerator *fg);	// Removes the given registered pair from the registry. If the pair is not registered, this method will have no effect.
							void							clear						();	// Clears all registrations from the registry. This will not delete the bodies or the force generators themselves, just the records of their connection.

							void							UpdateForces				(double duration);	// Calls all the force generators to update the forces of their corresponding bodies.
	};

	//// A force generator showing a three component explosion effect. This force generator is intended to represent a single explosion effect for multiple rigid bodies. The force generator can also act as a particle force generator.
	//class Explosion : public ForceGenerator,
	//                  public ParticleForceGenerator
	//{
	//    double									timePassed;	// Tracks how long the explosion has been in operation, used for time-sensitive effects.
	//public:
	//	// Properties of the explosion, these are public because there are so many and providing a suitable constructor would be cumbersome:
	//	Vector3									detonation					= {};	// The location of the detonation of the weapon.
	//	double									implosionMaxRadius			= 0;	// The radius up to which objects implode in the first stage of the explosion.
	//	double									implosionMinRadius			= 0;	// The radius within which objects don't feel the implosion force. Objects near to the detonation aren't sucked in by the air implosion.
	//	double									implosionDuration			= 0;	// The length of time that objects spend imploding before the concussion phase kicks in.
	//	double									implosionForce				= 0;	// The maximal force that the implosion can apply. This should be relatively small to avoid the implosion pulling objects through the detonation point and out the other side before the concussion wave kicks in.
	//	double									shockwaveSpeed				= 0;	// The speed that the shock wave is traveling, this is related to the thickness below in the relationship: thickness >= speed * minimum frame duration
	//	double									shockwaveThickness			= 0;	// The shock wave applies its force over a range of distances, this controls how thick. Faster waves require larger thicknesses.
	//	double									peakConcussionForce			= 0;	// This is the force that is applied at the very centre of the concussion wave on an object that is stationary. Objects that are in front or behind of the wavefront, or that are already moving outwards, get proportionally less force. Objects moving in towards the centre get proportionally more force.
	//	double									concussionDuration			= 0;	// The length of time that the concussion wave is active. As the wave nears this, the forces it applies reduces.
	//	double									peakConvectionForce			= 0;	// This is the peak force for stationary objects in the centre of the convection chimney. Force calculations for this value are the same as for peakConcussionForce.
	//	double									chimneyRadius				= 0;	// The radius of the chimney cylinder in the xz plane.
	//	double									chimneyHeight				= 0;	// The maximum height of the chimney.
	//	double									convectionDuration			= 0;	// The length of time the convection chimney is active. Typically this is the longest effect to be in operation, as the heat from the explosion outlives the shock wave and implosion itself.
	//
	//public:
	//    virtual void							UpdateForce					(RigidBody * body, double duration)		= 0;	// Calculates and applies the force that the explosion has on the given rigid body.
	//    virtual void							UpdateForce					(Particle *particle, double duration)	= 0;	// Calculates and applies the force that the explosion has on the given particle.
	//
	//};
}

#endif // CYCLONE_FGEN_H