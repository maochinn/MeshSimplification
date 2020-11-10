/************************************************************************
	 File:        CallBacks.H

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu
	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu

	 Comment:     Header file to define callback functions.
						define the callbacks for the TrainWindow

						these are little functions that get called when the
						various widgets
						get accessed (or the fltk timer ticks). these
						functions are used
						when TrainWindow sets itself up.

	 Platform:    Visio Studio.Net 2003/2005

*************************************************************************/
#pragma once

#include <time.h>
#include <math.h>

#include "MyWindow.h"
#include "MyView.h"
#include "CallBack.h"

#pragma warning(push)
#pragma warning(disable:4312)
#pragma warning(disable:4311)
#include <Fl/Fl_File_Chooser.H>
#include <Fl/math.h>
#pragma warning(pop)
//***************************************************************************
//
// * any time something changes, you need to force a redraw
//===========================================================================
void damageCB(Fl_Widget*, MyWindow* mw)
{
	mw->damageMe();
}

static unsigned long lastRedraw = 0;
//***************************************************************************
//
// * Callback for idling - if things are sitting, this gets called
// if the run button is pushed, then we need to make the train go.
// This is taken from the old "RunButton" demo.
// another nice problem to have - most likely, we'll be too fast
// don't draw more than 30 times per second
//===========================================================================
void idleCB(MyWindow* mw)
//===========================================================================
{
	if (mw != NULL)
	{
		//float fps = CLOCKS_PER_SEC / (float)(clock() - lastRedraw);
		//if (fps < 30.0)
		if (clock() - lastRedraw > CLOCKS_PER_SEC / 30) 
		{
			//system("cls");
			//std::cout << "FPS:" << fps << std::endl;

			lastRedraw = clock();
			mw->damageMe();
		}

	}
}

void testingCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->gl_mesh->degenerationMeshToLine(mw->degeneration_slider->value());
	mw->damageMe();
}

void exportCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->gl_mesh->exportSimplificationMesh(mw->simplification_slider->value());
	mw->damageMe();
}

void simplificationCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->gl_mesh->simplification(mw->simplification_slider->value());
	mw->damageMe();
}

void degenerationCB(Fl_Widget*, MyWindow* mw)
{
	mw->myView->gl_mesh->degenerateLeastSquareMesh(mw->degeneration_slider->value());
	mw->damageMe();
}


//void doCB(Fl_Widget*, MyWindow* mw)
//{
//	mw->myView->gl_mesh->collispe();
//	mw->damageMe();
//}
//void undoCB(Fl_Widget*, MyWindow* mw)
//{
//	mw->myView->gl_mesh->simplification();
//	mw->damageMe();
//}

