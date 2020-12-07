/************************************************************************
	 File:        MyWindow.cpp (from MyWindow.cpp)

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu

	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu
				  Maochinn, m10815023@gapps.edu.tw

	 Comment:
						this class defines the window in which the project
						runs - its the outer windows that contain all of
						the widgets, including the "TrainView" which has the
						actual OpenGL window in which the train is drawn

						You might want to modify this class to add new widgets
						for controlling	your train

						This takes care of lots of things - including installing
						itself into the FlTk "idle" loop so that we get periodic
						updates (if we're running the train).


	 Platform:    Visio Studio 2019

*************************************************************************/

#include <FL/fl.h>
#include <FL/Fl_Box.h>



// for using the real time clock
#include <time.h>

#include "MyWindow.h"
#include "MyView.h"
#include "CallBack.h"



//************************************************************************
//
// * Constructor
//========================================================================
MyWindow::
MyWindow(const int x, const int y)
	: Fl_Double_Window(x, y, 800, 600, "Mesh Simplification")
	//========================================================================
{
	// make all of the widgets
	begin();	// add to this widget
	{
		int pty = 5;			// where the last widgets were drawn

		myView = new MyView(5, 5, 590, 590);
		myView->mw = this;
		//trainView->m_pTrack = &m_Track;
		this->resizable(myView);

		// to make resizing work better, put all the widgets in a group
		widgets = new Fl_Group(600, 5, 190, 590);
		widgets->begin();

		renderMeshButton = new Fl_Button(605, pty, 60, 20, "Mesh");
		togglify(renderMeshButton, 1);

		renderSkeletonButton = new Fl_Button(730, pty, 65, 20, "Skeleton");
		togglify(renderSkeletonButton, 0);

		pty += 25;

		// camera buttons - in a radio button group
		Fl_Group* camGroup = new Fl_Group(600, pty, 195, 20);
		camGroup->begin();
		world_cam = new Fl_Button(605, pty, 60, 20, "World");
		world_cam->type(FL_RADIO_BUTTON);		// radio button
		world_cam->value(0);			// turned off
		world_cam->selection_color((Fl_Color)3); // yellow when pressed
		world_cam->callback((Fl_Callback*)damageCB, this);

		top_cam = new Fl_Button(735, pty, 60, 20, "Top");
		top_cam->type(FL_RADIO_BUTTON);
		top_cam->value(1);
		top_cam->selection_color((Fl_Color)3);
		top_cam->callback((Fl_Callback*)damageCB, this);
		camGroup->end();
		pty += 30;

		// browser to select spline types
		//simplification_browser = new Fl_Browser(605, pty, 120, 75, "Method");
		//simplification_browser->type(2);		// select
		//simplification_browser->callback((Fl_Callback*)damageCB, this);
		//simplification_browser->add("Average");
		//simplification_browser->add("Median");
		//simplification_browser->add("Error quadrics");
		//simplification_browser->select(1);

		//pty += 110;

		// reset the points
		Fl_Button* importMesh = new Fl_Button(605, pty, 60, 20, "Import");
		importMesh->callback((Fl_Callback*)importCB, this);
		Fl_Button* exportMesh = new Fl_Button(675, pty, 60, 20, "Export");
		exportMesh->callback((Fl_Callback*)exportCB, this);
		Fl_Button* reset = new Fl_Button(735, pty, 60, 20, "Reset");
		reset->callback((Fl_Callback*)resetCB, this);
		pty += 25;

		/*Fl_Button* Simplification = new Fl_Button(605, pty, 60, 20, "Simplification");
		Simplification->callback((Fl_Callback*)simplificationCB, this);
		pty += 25;*/

		/*simplification_slider = new Fl_Value_Slider(655, pty, 140, 20, "S");
		simplification_slider->range(0.0, 1.0);
		simplification_slider->value(1.0);
		simplification_slider->align(FL_ALIGN_LEFT);
		simplification_slider->type(FL_HORIZONTAL);
		simplification_slider->callback((Fl_Callback*)simplificationSlideCB, this);
		pty += 30;*/

		/*WL0 = new Fl_Value_Input(675, pty, 60, 20, "WL0"); WL0->value(0.001); pty += 25;
		WH0 = new Fl_Value_Input(675, pty, 60, 20, "WH0"); WH0->value(1.0); pty += 25;
		SL = new Fl_Value_Input(675, pty, 60, 20, "SL"); SL->value(4.0); pty += 25;*/
		

		/*Fl_Button* Skeleton = new Fl_Button(605, pty, 60, 20, "Skeleton");
		Skeleton->callback((Fl_Callback*)SkeletonCB, this);
		pty += 25;*/

#ifdef EXAMPLE_SOLUTION
		makeExampleWidgets(this, pty);
#endif

		// we need to make a little phantom widget to have things resize correctly
		Fl_Box* resizebox = new Fl_Box(600, 595, 200, 5);
		widgets->resizable(resizebox);

		widgets->end();
	}
	end();	// done adding to this widget

	// set up callback on idle
	Fl::add_idle((void (*)(void*))idleCB, this);
}

//************************************************************************
//
// * handy utility to make a button into a toggle
//========================================================================
void MyWindow::
togglify(Fl_Button* b, int val)
//========================================================================
{
	b->type(FL_TOGGLE_BUTTON);		// toggle
	b->value(val);		// turned off
	b->selection_color((Fl_Color)3); // yellow when pressed	
	b->callback((Fl_Callback*)damageCB, this);
}

//************************************************************************
//
// *
//========================================================================
void MyWindow::
damageMe()
//========================================================================
{
	myView->damage(1);
}