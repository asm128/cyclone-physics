// The definition file for the default application object.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "app.h"

#include "ogl_headers.h"
#include "timing.h"

#include <cstring>

void							Application::InitGraphics									()																{
	glClearColor	(0.9f, 0.95f, 1.0f, 1.0f);
	glEnable		(GL_DEPTH_TEST);
	glShadeModel	(GL_SMOOTH);
	SetView			();
}

void							Application::Update											()																{ glutPostRedisplay(); }
void							Application::Display										()																{
	glClear			(GL_COLOR_BUFFER_BIT);
	glBegin			(GL_LINES);
	glVertex2i		(1, 1);
	glVertex2i		(639, 319);
	glEnd			();
}

void							Application::SetView										()																{
	glMatrixMode	(GL_PROJECTION);
	glLoadIdentity	();
	gluPerspective	(60.0, (double)Width/(double)Height, 1.0, 500.0);
	glMatrixMode	(GL_MODELVIEW);
}


void							Application::Resize											(int width, int height)											{
	if (height <= 0)	// Avoid the divide by zero.
		height							= 1;

	// Set the internal variables and update the view
	Width							= width;
	Height							= height;
	glViewport(0, 0, width, height);
	SetView();
}

// The following methods aren't intended to be overloaded
void							Application::RenderText										(float x, float y, const char *text, void *font)				{
	glDisable		(GL_DEPTH_TEST);

	glMatrixMode	(GL_PROJECTION);	// Temporarily set up the view in orthographic projection.
	glPushMatrix	();
	glLoadIdentity	();
	glOrtho			(0.0, (double)Width, 0.0, (double)Height, -1.0, 1.0);

	glMatrixMode	(GL_MODELVIEW);		// Move to modelview mode.
	glPushMatrix	();
	glLoadIdentity	();

	if (font == NULL)	// Ensure we have a font
		font							= GLUT_BITMAP_HELVETICA_10;

	size_t								len															= ::strlen(text);
	glRasterPos2f	(x, y);
	for (const char *letter = text; letter < text+len; letter++) {	// Loop through characters displaying them.
		if (*letter == '\n') {	// If we meet a newline, then move down by the line-height TODO: Make the line-height a function of the font
			y								-= 12.0f;
			glRasterPos2f(x, y);
		}
		glutBitmapCharacter(font, *letter);
	}

	// Pop the matrices to return to how we were before.
	glPopMatrix		();
	glMatrixMode	(GL_PROJECTION);
	glPopMatrix		();
	glMatrixMode	(GL_MODELVIEW);
	glEnable		(GL_DEPTH_TEST);
}

								MassAggregateApplication::MassAggregateApplication			(uint32_t particleCount)
	: World(particleCount*10)
{
	ParticleArray					= new cyclone::Particle[particleCount];
	for (uint32_t i = 0; i < particleCount; i++)
		World.Particles.push_back(ParticleArray + i);

	GroundContactGenerator.Init(&World.Particles);
	World.ContactGenerators.push_back(&GroundContactGenerator);
}

void							MassAggregateApplication::Display							()																{
	// Clear the view port and set the camera direction
	glClear			(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity	();
	gluLookAt		(0.0, 3.5, 8.0,  0.0, 3.5, 0.0,  0.0, 1.0, 0.0);

	glColor3f		(0,0,0);

	cyclone::ParticleWorld::TParticles &particles = World.Particles;
	for (cyclone::ParticleWorld::TParticles::iterator p = particles.begin(); p != particles.end(); ++p) {
		cyclone::Particle					* particle													= *p;
		const cyclone::Vector3				& pos														= particle->Position;
		glPushMatrix	();
		glTranslatef	(pos.x, pos.y, pos.z);
		glutSolidSphere	(0.1f, 20, 10);
		glPopMatrix		();
	}
}

void							MassAggregateApplication::Update							()																{
	World.startFrame();	// Clear accumulators
	float								duration													= (float)TimingData::get().LastFrameDuration * 0.001f;	// Find the duration of the last frame in seconds
	if (duration <= 0.0f) 
		return;

	World.runPhysics(duration);	// Run the simulation
	Application::Update();
}

void							RigidBodyApplication::Update								()																{
	float								duration													= (float)TimingData::get().LastFrameDuration * 0.001f;	// Find the duration of the last frame in seconds
	if (duration <= 0.0f) 
		return;	// nothing to update since no time has passed 
	else if (duration > 0.05f) 
		duration						= 0.05f;

	if(PauseSimulation) {	// Exit immediately if we aren't running the simulation
		Application::Update();
		return;
	}
	else if (AutoPauseSimulation) {
		PauseSimulation					= true;
		AutoPauseSimulation				= false;
	}
	UpdateObjects		(duration);															// Update the objects
	GenerateContacts	();																	// Perform the contact generation
	Resolver.resolveContacts(Collisions.ContactArray, Collisions.ContactCount, duration);	// Resolve detected contacts

	Application::Update();
}

void							RigidBodyApplication::Display								()																{
	glClear			(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity	();
	gluLookAt		(18.0f, 0, 0,  0, 0, 0,  0, 1.0f, 0);
	glRotatef		(-CameraPhi, 0, 0, 1);
	glRotatef		(CameraTheta, 0, 1, 0);
	glTranslatef	(0, -5.0f, 0);
}

void							RigidBodyApplication::DrawDebug								()																{
	if (!RenderDebugInfo) 
		return;
	GenerateContacts();	// Recalculate the contacts, so they are current (in case we're paused, for example).

	// Render the contacts, if required
	glBegin	(GL_LINES);
	for (uint32_t i = 0; i < Collisions.ContactCount; i++) {
		if (Contacts[i].Body[1])	// Interbody contacts are in green, floor contacts are red.
			glColor3f	(0,1,0);
		else
			glColor3f	(1,0,0);

		cyclone::Vector3					vec															= Contacts[i].ContactPoint;
		glVertex3f	(vec.x, vec.y, vec.z);

		vec								+= Contacts[i].ContactNormal;
		glVertex3f	(vec.x, vec.y, vec.z);
	}
	glEnd	();
}


void							RigidBodyApplication::MouseDrag								(int x, int y)													{
	CameraTheta						+= (x - Last_x) * 0.25f;	// Update the camera
	CameraPhi						+= (y - Last_y) * 0.25f;

	// Keep it in bounds
		 if (CameraPhi < -20.0f) CameraPhi = -20.0f;
	else if (CameraPhi >  80.0f) CameraPhi = 80.0f;

	Last_x = x;	// Remember the position
	Last_y = y;
}

void							RigidBodyApplication::Key									(unsigned char key)												{
	switch(key) {
	case 'R': case 'r': Reset();							return; // Reset the simulation
	case 'C': case 'c': RenderDebugInfo	= !RenderDebugInfo;	return; // Toggle rendering of contacts
	case 'P': case 'p': PauseSimulation	= !PauseSimulation; return; // Toggle running the simulation
	case ' ':	// Advance one frame
		AutoPauseSimulation				= true;
		PauseSimulation					= false;
	}

	Application::Key(key);
}
