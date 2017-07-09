// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "pfgen.h"

using namespace cyclone;

void ParticleForceRegistry::UpdateForces(double duration)
{
    TRegistry::iterator i = registrations.begin();
    for (; i != registrations.end(); i++)
        i->fg->UpdateForce(i->particle, duration);
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg)
{
    ParticleForceRegistry::ParticleForceRegistration registration;
    registration.particle = particle;
    registration.fg = fg;
    registrations.push_back(registration);
}


void ParticleGravity::UpdateForce(Particle* particle, double duration) {
    if (!particle->HasFiniteMass())		// Check that we do not have infinite mass
		return;
    particle->AccumulatedForce			+= Gravity * particle->GetMass();	// Apply the mass-scaled force to the particle
}

void ParticleDrag::UpdateForce(Particle* particle, double duration)
{
    Vector3 force = particle->Velocity;

    // Calculate the total drag coefficient
    double dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    // Calculate the final force and apply it
    force.normalise();
    force *= -dragCoeff;
    particle->AccumulatedForce			+= force;
}



void ParticleSpring::UpdateForce(Particle* particle, double duration)
{
    // Calculate the vector of the spring
    Vector3 force = particle->Position;
    force -= Other->Position;

    // Calculate the magnitude of the force
    double magnitude = force.magnitude();
    magnitude = real_abs(magnitude - RestLength);
    magnitude *= SpringConstant;

    // Calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    particle->AccumulatedForce			+= force;
}

void	ParticleBuoyancy::UpdateForce(Particle* particle, double duration)
{
    // Calculate the submersion depth
    double depth = particle->Position.y;

    // Check if we're out of the water
    if (depth >= WaterHeight + MaxDepth) 
		return;
	Vector3 force = {};

    // Check if we're at maximum depth
    if (depth <= WaterHeight - MaxDepth) {
        force.y = LiquidDensity * Volume;
        particle->AccumulatedForce			+= force;
        return;
    }

    // Otherwise we are partly submerged
    force.y								= LiquidDensity * Volume * (depth - MaxDepth - WaterHeight) / 2 * MaxDepth;
    particle->AccumulatedForce			+= force;
}


void	ParticleBungee::UpdateForce(Particle* particle, double duration) {
    Vector3		force		= particle->Position - Other->Position;	// Calculate the vector of the spring
	double		magnitude	= force.magnitude();
    if (magnitude <= RestLength)	// Check if the bungee is compressed
		return;

    magnitude = SpringConstant * (RestLength - magnitude);	// Calculate the magnitude of the force

    // Calculate the final force and apply it
    force.normalise();
    force *= -magnitude;
    particle->AccumulatedForce			+= force;
}


void	ParticleFakeSpring::UpdateForce			(Particle* particle, double duration)			{
    if (!particle->HasFiniteMass())     // Check that we do not have infinite mass
		return;

    // Calculate the relative position of the particle to the anchor
    Vector3 position = particle->Position;
    position -= *Anchor;

    // Calculate the constants and check they are in bounds.
    double gamma = 0.5f * real_sqrt(4 * SpringConstant - Damping*Damping);
    if (gamma == 0.0f) 
		return;
    Vector3 c = position * (Damping / (2.0f * gamma)) +
        particle->Velocity * (1.0f / gamma);

    // Calculate the target position
    Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
    target *= real_exp(-0.5f * duration * Damping);

    // Calculate the resulting acceleration and therefore the force
    Vector3 accel = (target - position) * ((double)1.0 / (duration*duration)) - particle->Velocity * ((double)1.0/duration);
    particle->AccumulatedForce			+= accel * particle->GetMass();
}

void ParticleAnchoredSpring::Init(Vector3 *anchor, double springConstant, double restLength) {
    Anchor			= anchor;
    SpringConstant	= springConstant;
    RestLength		= restLength;
}

void ParticleAnchoredBungee::UpdateForce(Particle* particle, double duration)
{
	// Calculate the vector of the spring
	Vector3									force						= particle->Position;
	force								-= *Anchor;

	double									magnitude					= force.magnitude();    // Calculate the magnitude of the force
	if (magnitude < RestLength) 
		return;

	magnitude							= magnitude - RestLength;
	magnitude							*= SpringConstant;

	// Calculate the final force and apply it
	force.normalise();
	force								*= -magnitude;
	particle->AccumulatedForce			+= force;
}

void ParticleAnchoredSpring::UpdateForce(Particle* particle, double duration)
{
    // Calculate the vector of the spring
    Vector3 force = particle->Position;
    force -= *Anchor;

    // Calculate the magnitude of the force
    double magnitude = force.magnitude();
    magnitude = (RestLength - magnitude) * SpringConstant;
	
    // Calculate the final force and apply it
    force.normalise();
    force								*= magnitude;
    particle->AccumulatedForce			+= force;
}
