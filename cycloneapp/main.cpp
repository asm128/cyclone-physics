// The main entry point for all demos.
// Copyright (c) Icosagon 2003. Published by Ian Millington under the MIT License for his book "Game Physics Engine Development" or something like that (a really good book that I actually bought in paperback after reading it).
// Heavily modified by asm128 in order to make this code readable and free of potential bugs and inconsistencies and a large set of sources of problems and improductivity originally introduced thanks to poor advice, bad practices and OOP vices.
#include "app.h"			// Include the general application structure.
#include "ogl_headers.h"	// Include appropriate OpenGL headers.
#include "timing.h"			// Include the timing functions

extern Application*		getApplication			();	// Forward declaration of the function that will return the application object for this particular demo. This should be implemented in the demo's .cpp file.

Application				* app					= nullptr;	// Store the global application object.


void					mouse					(int button, int state, int x, int y)				{ app->Mouse(button, state, x, y);	}	// Called when a mouse button is pressed. Delegates to the application.
void					reshape					(int width, int height)								{ app->Resize(width, height);		}	// Called when the display window changes size.
void					keyboard				(unsigned char key, int /*x*/, int /*y*/)			{ app->Key(key);					}	// Called when a key is pressed. Note we omit passing on the x and y: they are rarely needed.
void					motion					(int x, int y)										{ app->MouseDrag(x, y);				}	// Called when the mouse is dragged.
void					createWindow			(const char* title)									{	// Creates a window in which to display the scene.
	glutInitDisplayMode		(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize		(640,320);
	glutInitWindowPosition	(0,0);
	glutCreateWindow		(title);
}

void					update					()													{	// Called each frame to update the 3D scene. Delegates to the application.
	TimingData::get().update();	// Update the timing.
	app->Update();				// Delegate to the application.
}

void					display					()													{	// Called each frame to display the 3D scene. Delegates to the application.
	app->Display();
	// Update the displayed content.
	glFlush();
	glutSwapBuffers();
}

// The main entry point. We pass arguments onto GLUT.
int						main					(int argc, char** argv)								{
	// Set up GLUT and the timers
	glutInit(&argc, argv);
	TimingData::init();
	
	// Create the application and its window
	app						= getApplication();
	createWindow(app->GetTitle());
	
	// Set up the appropriate handler functions
	glutReshapeFunc		(reshape);
	glutKeyboardFunc	(keyboard);
	glutDisplayFunc		(display);
	glutIdleFunc		(update);
	glutMouseFunc		(mouse);
	glutMotionFunc		(motion);
	
	// Run the application
	app->InitGraphics();
	glutMainLoop();
	
	// Clean up the application
	app->Deinit();
	delete app;
	TimingData::deinit();
}
