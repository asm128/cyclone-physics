// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "cyclone.h"

#include <cstdlib>

// An application is the base class for all demonstration progams. 
// GLUT is a c-style API, which calls bare functions. This makes it more difficult to provide default services for all demos and only override them when needed.
// To solve this, the GLUT API is translated into calls on a generic application object. Each demonstration will create a concrete subclass of Application, providing the behaviours it needs. 
// The common code for all demos manages dispatch of requests to the appropriate application object.
// To provide a correct application object of the right type without the core code needing to know which subclass is being used, each demonstration will supply a getApplication function which creates (with new) and returns a pointer to a new Application instance.
// Even though subclasses will have to implement most of the methods in this class, I have not made them pure virtual. This saves the annoying need to implement an empty function that isn't needed.
class Application{
protected:
	int								Height;	// Holds the height of the application window.
	int								Width;	// Holds the current width of the application window.

public:
	virtual const char*				GetTitle							();	// Gets the title of the demo for the title bar of the window. The default implementation returns a generic title.
	virtual void					InitGraphics						();	// Sets up the graphics, and allows the application to acquire graphical resources. Guaranteed to be called after OpenGL is set up. The default implementation sets up a basic view, and calls setView to set up the camera projection.
	virtual void					SetView								();	// Called to set the projection characteristics of the camera. The default implementation uses a 60 degree field of view camera with a range from 1-500 units.
	virtual void					Deinit								()										{}	// Called just before the application is destroyed. Clear up can be performed here or in the application destructor. The default implementation does nothing.

    // Called each frame to display The current scene. The common code will automatically flush the graphics pipe and swap the render buffers after calling this so glFlush doesn't need to be called.
    // The default implementation drAws a simple diagonal line across the surface (as a sanity check to make sure GL is working).
    virtual void					Display								();

    // Called each frame to update tHe current state of the scene. The default implementation requests that the display be refreshed. It should probably be called from any subclass update as the last command.
    virtual void					Update								();

	// Notifies the application that the window has changed size. The new size is given.
	// The default implementation seTs the internal height and width parameters and changes the gl viewport. These are steps you'll almost always need, so its worth calling the base class version of this method even if you override it in a demo class.
	virtual void					Resize								(int width, int height);

	virtual void					Key									(unsigned char asciiCode)				{ asciiCode; }	// Called when a keypress is detected.
	virtual void					Mouse								(int button, int state, int x, int y)	= 0;	// Called when GLUT detects a mouse button press.
	virtual void					MouseDrag							(int x, int y)							= 0;	// Called when GLUT detects a mouse drag.

	// --- These are helper functionS that can be used by an application to render things.
	void							RenderText							(float x, float y, const char * text, void * font = NULL);	// Renders the given text to the given x,y location (in screen space) on the window. This is used to pass status information to the application.
};

// This application adds additional functionality used in the mass-aggregate demos.
class MassAggregateApplication : public Application {
protected:
	cyclone::ParticleWorld			World;
	cyclone::Particle				* ParticleArray;
	cyclone::GroundContacts			GroundContactGenerator;

public:
	virtual							~MassAggregateApplication			();
									MassAggregateApplication			(unsigned int particleCount);

	virtual void					Update								();	// Update the particle positions.
	virtual void					InitGraphics						();	// Sets up the graphic rendering. 
	virtual void					Display								();	// Display the particles.
};

// This application adds additional functionality used in many of the demos. This includes the ability to track contacts (for rigid bodies) and move the camera around.
class RigidBodyApplication : public Application {
protected:
	static const uint32_t			MaxContacts							= 256;	// Holds the maximum number of contacts.
	cyclone::Contact				Contacts[MaxContacts]				= {};	// Holds the array of contacts.
	cyclone::CollisionData			CData;										// Holds the collision data structure for collision detection.
	cyclone::ContactResolver		Resolver;									// Holds the contact resolver.
	float							Theta;										// Holds the camera angle.
	float							Phi;										// Holds the camera elevation.
	int								Last_x, Last_y;								// Holds the position of the mouse at the last frame of a drag.
	bool							RenderDebugInfo;							// True if the contacts should be rendered.
	bool							PauseSimulation;							// True if the simulation is paused.
	bool							AutoPauseSimulation;						// Pauses the simulation after the next frame automatically.
	
	virtual void					GenerateContacts					()										= 0;	// Processes the contact generation code.
	virtual void					UpdateObjects						(cyclone::real duration)				= 0;	// Processes the objects in the simulation forward in time. */
	virtual void					Reset								()										= 0;	// Resets the simulation.
	void							DrawDebug							();												// Finishes drawing the frame, adding debugging information as needed.
public:
	
									RigidBodyApplication				();	// Creates a new application object.

	virtual void					Display								();	// Display the application.
	virtual void					Update								();	// Update the objects.	
	virtual void					Mouse								(int button, int state, int x, int y);	// Handle a mouse click.
	virtual void					MouseDrag							(int x, int y);							// Handle a mouse drag.
	virtual void					Key									(unsigned char key);					// Handles a key press.
};
