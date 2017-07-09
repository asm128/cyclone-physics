// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "body.h"

#ifndef CYCLONE_CONTACTS_H
#define CYCLONE_CONTACTS_H

namespace cyclone {
	 // A contact represents two bodies in contact. Resolving a contact removes their interpenetration, and applies sufficient impulse to keep them apart. Colliding bodies may also rebound.
	 // Contacts can be used to represent positional joints, by making the contact constraint keep the bodies in their correct orientation.
	 //
	 // It can be a good idea to create a contact object even when the contact isn't violated. Because resolving one contact can violate another, contacts that are close to being violated should be sent to the resolver; 
	 // that way if one resolution moves the body, the contact may be violated, and can be resolved. If the contact is not violated, it will not be resolved, so you only loose a small amount of execution time.
	struct Contact {
		RigidBody			* Body[2]							= {};	// Holds the bodies that are involved in the contact. The second of these can be NULL, for contacts with the scenery.
		double				Friction							= 0;	// Holds the lateral friction coefficient at the contact.
		double				Restitution							= 0;	// Holds the normal restitution coefficient at the contact.
		Vector3				ContactPoint						= {};	// Holds the position of the contact in world coordinates.
		Vector3				ContactNormal						= {};	// Holds the direction of the contact in world coordinates.
		double				Penetration							= 0;	// Holds the depth of penetration at the contact point. If both bodies are specified then the contact point should be midway between the inter-penetrating points.
		
		void				setBodyData							(RigidBody* one, RigidBody *two, double friction, double restitution);// Sets the data that doesn't normally depend on the position of the contact (i.e. the bodies, and their material properties).
	
	//protected:
		Matrix3				ContactToWorld						= {};	// A transform matrix that converts co-ordinates in the contact's frame of reference to world co-ordinates. The columns of this matrix form an orthonormal set of vectors.
		Vector3				ContactVelocity						= {};	// Holds the closing velocity at the point of contact. This is set when the calculateInternals function is run.
		double				DesiredDeltaVelocity				= 0;	// Holds the required change in velocity for this contact to be resolved.
		Vector3				RelativeContactPosition[2]			= {};	// Holds the world space position of the contact point relative to centre of each body. This is set when the calculateInternals function is run.
	
		void				calculateInternals					(double duration);						// Calculates internal data from state data. This is called before the resolution algorithm tries to do any resolution. It should never need to be called manually.
		void				swapBodies							();										// Reverses the contact. This involves swapping the two rigid bodies and reversing the contact normal. The internal values should then be recalculated using calculateInternals (this is not done automatically).
		void				matchAwakeState						();										// Updates the awake state of rigid bodies that are taking place in the given contact. A body will be made awake if it is in contact with a body that is awake.
		void				calculateDesiredDeltaVelocity		(double duration);						// Calculates and sets the internal value for the desired delta velocity.
		Vector3				calculateLocalVelocity				(uint32_t bodyIndex, double duration);	// Calculates and returns the velocity of the contact point on the given body.
		void				calculateContactBasis				();										// Calculates an orthonormal basis for the contact point, based on the primary friction direction (for anisotropic friction) or a random orientation (for isotropic friction).
		//void				applyImpulse						(const Vector3 &impulse, RigidBody *body, Vector3 *velocityChange, Vector3 *rotationChange);	// Applies an impulse to the given body, returning the change in velocities.
		void				applyVelocityChange					(Vector3 velocityChange[2], Vector3 rotationChange[2]);	// Performs an inertia-weighted impulse based resolution of this contact alone.
		
		
		void				applyPositionChange					(Vector3 linearChange[2], Vector3 angularChange[2], double penetration);	// Performs an inertia weighted penetration resolution of this contact alone.
		Vector3				calculateFrictionlessImpulse		(Matrix3 *inverseInertiaTensor);	// Calculates the impulse needed to resolve this contact, given that the contact has no friction. A pair of inertia tensors - one for each contact object - is specified to save calculation time: the calling function has access to these anyway.
	
		// Calculates the impulse needed to resolve this contact, given that the contact has a non-zero coefficient of friction. 
		// A pair of inertia tensors - one for each contact object - is specified to save calculation time: the calling function has access to these anyway.
		Vector3				calculateFrictionImpulse			(Matrix3 *inverseInertiaTensor);
	};
	
	// The contact resolution routine. One resolver instance can be shared for the whole simulation, as long as you need roughly the same parameters each time (which is normal).
	//
	// --- Resolution Algorithm
	//
	// The resolver uses an iterative satisfaction algorithm; it loops through each contact and tries to resolve it. Each contact is resolved locally, which may in turn put other contacts in a worse position. 
	// The algorithm then revisits other contacts and repeats the process up to a specified iteration limit. It can be proved that given enough iterations, the simulation will get to the correct result. 
	// As with all approaches, numerical stability can cause problems that make a correct resolution impossible.
	//
	// --- Strengths
	//
	// This algorithm is very fast, much faster than other physics approaches. Even using many more iterations than there are contacts, it will be faster than global approaches.
	// Many global algorithms are unstable under high friction, this approach is very robust indeed for high friction and low restitution values.
	// The algorithm produces visually believable behaviour. Tradeoffs have been made to err on the side of visual realism rather than computational expense or numerical accuracy.
	//
	// --- Weaknesses
	//
	// The algorithm does not cope well with situations with many inter-related contacts: stacked boxes, for example. In this case the simulation may appear to jiggle slightly, which often dislodges a box from the stack, allowing it to collapse.
	// Another issue with the resolution mechanism is that resolving one contact may make another contact move sideways against friction, because each contact is handled independently, this friction is not taken into account. 
	// If one object is pushing against another, the pushed object may move across its support without friction, even though friction is set between those bodies.
	// In general this resolver is not suitable for stacks of bodies, but is perfect for handling impact, explosive, and flat resting situations.
	class ContactResolver {
	protected:
		uint32_t			VelocityIterations					= 0;	// Holds the number of iterations to perform when resolving velocity.
		uint32_t			PositionIterations					= 0;	// Holds the number of iterations to perform when resolving position.
		double				VelocityEpsilon						= 0.01;	// To avoid instability velocities smaller than this value are considered to be zero. Too small and the simulation may be unstable, too large and the bodies may interpenetrate visually. A good starting point is the default of 0.01.
		double				PositionEpsilon						= 0.01;	// To avoid instability penetrations smaller than this value are considered to be not interpenetrating. Too small and the simulation may be unstable, too large and the bodies may interpenetrate visually. A good starting point is the default of0.01.

	public:
		uint32_t			VelocityIterationsUsed				= 0;	// Stores the number of velocity iterations used in the last call to resolve contacts.
		uint32_t			PositionIterationsUsed				= 0;	// Stores the number of position iterations used in the last call to resolve contacts.

	private:
		bool				ValidSettings						= false;	// Keeps track of whether the internal settings are valid.

	public:

							ContactResolver						(uint32_t iterations, double velocityEpsilon = 0.01, double positionEpsilon = 0.01) 
			: VelocityIterations	(iterations)
			, PositionIterations	(iterations)
			, VelocityEpsilon		(velocityEpsilon)
			, PositionEpsilon		(positionEpsilon)
			{}
							ContactResolver						(uint32_t velocityIterations, uint32_t positionIterations, double velocityEpsilon = 0.01, double positionEpsilon = 0.01) 
			: VelocityIterations	(velocityIterations)
			, PositionIterations	(positionIterations)
			, VelocityEpsilon		(velocityEpsilon)
			, PositionEpsilon		(positionEpsilon)
			{}

		// Returns true if the resolver has valid settings and is ready to go.
		inline void			setIterations						(uint32_t	iterations)														{ setIterations(iterations, iterations);	}	// Sets the number of iterations for both resolution stages.
		inline void			setIterations						(uint32_t	velocityIterations	, uint32_t	positionIterations	)			{ VelocityIterations	= velocityIterations	; PositionIterations	= positionIterations	; }					// Sets the number of iterations for each resolution stage.
		inline void			setEpsilon							(double		velocityEpsilon		, double	positionEpsilon		)			{ VelocityEpsilon		= velocityEpsilon		; PositionEpsilon		= positionEpsilon		; }							// Sets the tolerance value for both velocity and position.
		bool				isValid								()																			{
			return (VelocityIterations > 0) 
				&& (PositionIterations > 0) 
				&& (PositionEpsilon >= 0.0f)
				&& (PositionEpsilon >= 0.0f)
				;
		}
		// Resolves a set of contacts for both penetration and velocity.
		// Contacts that cannot interact with each other should be passed to separate calls to resolveContacts, as the resolution algorithm takes much longer for lots of contacts than it does for the same number of contacts in small sets.
		// @param numIterations The number of iterations through the resolution algorithm. This should be at least the number of contacts (otherwise some constraints will not be resolved - although sometimes this is not noticable). 
		// If the iterations are not needed they will not be used, so adding more iterations may not make any difference. In some cases you would need millions of iterations. 
		// Think about the number of iterations as a bound: if you specify a large number, sometimes the algorithm WILL use it, and you may drop lots of frames.
		// 
		// @param duration The duration of the previous integration step. This is used to compensate for forces applied.
		void				resolveContacts						(Contact *contactArray	, uint32_t numContacts, double duration);

	protected:
		void				prepareContacts						(Contact *contactArray	, uint32_t numContacts, double duration);	// Sets up contacts ready for processing. This makes sure their internal data is configured correctly and the correct set of bodies is made alive.
		void				adjustVelocities					(Contact *contactArray	, uint32_t numContacts, double duration);	// Resolves the velocity issues with the given array of constraints, using the given number of iterations.
		void				adjustPositions						(Contact *contacts		, uint32_t numContacts, double duration);	// Resolves the positional issues with the given array of constraints, using the given number of iterations.
	};

	// This is the basic polymorphic interface for contact generators applying to rigid bodies.
	class ContactGenerator {
	public:
		// Fills the given contact structure with the generated contact. The contact pointer should point to the first available contact in a contact array, where limit is the maximum number of contacts in the array that can be written to. 
		// The method returns the number of contacts that have been written.
		virtual uint32_t	AddContact							(Contact *contact, uint32_t limit)									const	= 0;
	};

	// Joints link together two rigid bodies and make sure they do not separate. In a general phyiscs engine there may be many different types of joint, that reduce the number of relative degrees of freedom between two objects. 
	// This joint is a common position joint: each object has a location (given in body-coordinates) that will be kept at the same point in the simulation.
	struct Joint : public ContactGenerator {
		RigidBody			* Body		[2]						= {};	// Holds the two rigid bodies that are connected by this joint.
		Vector3				Position	[2]						= {};	// Holds the relative location of the connection for each body, given in local coordinates.
		double				Error								= 0;	// Holds the maximum displacement at the joint before the joint is considered to be violated. This is normally a small, epsilon value. It can be larger, however, in which case the joint will behave as if an inelastic cable joined the bodies at their joint locations.

		void				Set									( RigidBody *a, const Vector3& a_pos, RigidBody *b, const Vector3& b_pos, double error);	// Configures the joint in one go.
		uint32_t			AddContact							(Contact *contact, uint32_t limit) const;	// Generates the contacts required to restore the joint if it has been violated.
	};
} // namespace cyclone

#endif // CYCLONE_CONTACTS_H
