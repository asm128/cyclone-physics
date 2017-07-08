// The definition file for the default application object.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include <cstring>
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

void Application::InitGraphics()
{
    glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    SetView();
}

void Application::SetView()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)Width/(double)Height, 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void Application::Display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);
    glVertex2i(1, 1);
    glVertex2i(639, 319);
    glEnd();
}

const char* Application::GetTitle()
{
    return "Cyclone Demo";
}


void Application::Update()
{
    glutPostRedisplay();
}


void Application::Resize(int width, int height)
{
    // Avoid the divide by zero.
    if (height <= 0) 
		height = 1;

    // Set the internal variables and update the view
    Width				= width;
    Height				= height;
    glViewport(0, 0, width, height);
    SetView();
}

// The following methods aren't intended to be overloaded
void Application::RenderText(float x, float y, const char *text, void *font)
{
    glDisable(GL_DEPTH_TEST);

    // Temporarily set up the view in orthographic projection.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, (double)Width, 0.0, (double)Height, -1.0, 1.0);

    // Move to modelview mode.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Ensure we have a font
    if (font == NULL) {
        font = GLUT_BITMAP_HELVETICA_10;
    }

    // Loop through characters displaying them.
    size_t len = strlen(text);

    glRasterPos2f(x, y);
    for (const char *letter = text; letter < text+len; letter++) {

        // If we meet a newline, then move down by the line-height
        // TODO: Make the line-height a function of the font
        if (*letter == '\n') {
            y -= 12.0f;
            glRasterPos2f(x, y);
        }
        glutBitmapCharacter(font, *letter);
    }

    // Pop the matrices to return to how we were before.
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}


MassAggregateApplication::MassAggregateApplication(unsigned int particleCount)
:
World(particleCount*10)
{
    ParticleArray = new cyclone::Particle[particleCount];
    for (unsigned i = 0; i < particleCount; i++)
    {
        World.getParticles().push_back(ParticleArray + i);
    }

    GroundContactGenerator.Init(&World.getParticles());
    World.getContactGenerators().push_back(&GroundContactGenerator);
}

MassAggregateApplication::~MassAggregateApplication()
{
    delete[] ParticleArray;
}

void MassAggregateApplication::InitGraphics()
{
    // Call the superclass
    Application::InitGraphics();
}

void MassAggregateApplication::Display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 3.5, 8.0,  0.0, 3.5, 0.0,  0.0, 1.0, 0.0);

    glColor3f(0,0,0);

    cyclone::ParticleWorld::TParticles &particles = World.getParticles();
    for (cyclone::ParticleWorld::TParticles::iterator p = particles.begin();
        p != particles.end();
        p++)
    {
        cyclone::Particle *particle = *p;
        const cyclone::Vector3 &pos = particle->Position;
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glutSolidSphere(0.1f, 20, 10);
        glPopMatrix();
    }
}

void MassAggregateApplication::Update()
{
    // Clear accumulators
    World.startFrame();

    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Run the simulation
    World.runPhysics(duration);

    Application::Update();
}

RigidBodyApplication::RigidBodyApplication()
	:	Theta				(0.0f)
	,	Phi					(15.0f)
	,	Resolver			(MaxContacts*8)
	,	RenderDebugInfo		(false)
	,	PauseSimulation		(true)
	,	AutoPauseSimulation	(false)
{
    CData.contactArray = Contacts;
}

void RigidBodyApplication::Update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;
    else if (duration > 0.05f) duration = 0.05f;

    // Exit immediately if we aren't running the simulation
    if (PauseSimulation)
    {
        Application::Update();
        return;
    }
    else if (AutoPauseSimulation)
    {
        PauseSimulation = true;
        AutoPauseSimulation = false;
    }

    // Update the objects
    UpdateObjects(duration);

    // Perform the contact generation
    GenerateContacts();

    // Resolve detected contacts
    Resolver.resolveContacts(
        CData.contactArray,
        CData.contactCount,
        duration
        );

    Application::Update();
}

void RigidBodyApplication::Display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(18.0f, 0, 0,  0, 0, 0,  0, 1.0f, 0);
    glRotatef(-Phi, 0, 0, 1);
    glRotatef(Theta, 0, 1, 0);
    glTranslatef(0, -5.0f, 0);
}

void RigidBodyApplication::DrawDebug()
{
    if (!RenderDebugInfo) return;

    // Recalculate the contacts, so they are current (in case we're
    // paused, for example).
    GenerateContacts();

    // Render the contacts, if required
    glBegin(GL_LINES);
    for (unsigned i = 0; i < CData.contactCount; i++)
    {
        // Interbody contacts are in green, floor contacts are red.
        if (Contacts[i].body[1]) {
            glColor3f(0,1,0);
        } else {
            glColor3f(1,0,0);
        }

        cyclone::Vector3 vec = Contacts[i].contactPoint;
        glVertex3f(vec.x, vec.y, vec.z);

        vec += Contacts[i].contactNormal;
        glVertex3f(vec.x, vec.y, vec.z);
    }

    glEnd();
}

void RigidBodyApplication::Mouse(int button, int state, int x, int y)
{
    // Set the position
    Last_x = x;
    Last_y = y;
}

void RigidBodyApplication::MouseDrag(int x, int y)
{
    // Update the camera
    Theta	+= (x - Last_x)*0.25f;
    Phi		+= (y - Last_y)*0.25f;

    // Keep it in bounds
		 if (Phi < -20.0f) Phi = -20.0f;
    else if (Phi >  80.0f) Phi = 80.0f;

    // Remember the position
    Last_x = x;
    Last_y = y;
}

void RigidBodyApplication::Key(unsigned char key) {
    switch(key) {
    case 'R': case 'r': Reset();							return; // Reset the simulation
    case 'C': case 'c': RenderDebugInfo = !RenderDebugInfo;	return; // Toggle rendering of contacts
    case 'P': case 'p': PauseSimulation = !PauseSimulation; return; // Toggle running the simulation
    case ' ':	// Advance one frame
        AutoPauseSimulation = true;
        PauseSimulation		= false;
    }

    Application::Key(key);
}
