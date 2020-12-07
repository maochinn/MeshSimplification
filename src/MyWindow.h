/************************************************************************
	 File:        MyWindow.h (from TrainWindow.H)

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
#pragma once

#pragma warning(push)
#pragma warning(disable:4312)
#pragma warning(disable:4311)
#include <Fl/Fl_Double_Window.h>
#include <Fl/Fl_Button.h>
#include <Fl/Fl_Group.H>
#include <Fl/Fl_Value_Slider.H>
#include <Fl/Fl_Browser.H>
#include <FL/Fl_Value_Input.H>
#pragma warning(pop)

// we need to know what is in the world to show
//#include "Track.h"

// other things we just deal with as pointers, to avoid circular references
class MyView;

// if we're also making the sample solution, then we need to know 
// about the stuff we don't tell students
#ifdef EXAMPLE_SOLUTION
#include "TrainExample/TrainExample.H"
#endif

class MyWindow : public Fl_Double_Window {
public:
	MyWindow(const int x = 50, const int y = 50);
public:
	// call this method when things change
	void damageMe();

	// simple helper function to set up a button
	void togglify(Fl_Button*, int state = 0);
public:
	// keep track of the stuff in the world
	//CTrack				m_Track;

	// the widgets that make up the Window
	MyView* myView;

	Fl_Group* widgets;	// all widgets, grouped for resizing ease

	// utility buttons
	Fl_Button* renderMeshButton;
	Fl_Button* renderSkeletonButton;

	// which viewpoint are we drawing from
	Fl_Button* world_cam;
	Fl_Button* top_cam;

	Fl_Browser* simplification_browser;

	//Fl_Value_Slider* simplification_slider;
	//Fl_Value_Input* WL0;
};
