/************************************************************************
	 File:        MyView.cpp (From MyView.cpp)

	 Author:
				  Michael Gleicher, gleicher@cs.wisc.edu

	 Modifier
				  Yu-Chi Lai, yu-chi@cs.wisc.edu
				  Maochinn, m10815023@gapps.ntust.edu

	 Comment:
						The MyView is the window that actually shows the
						train. Its a
						GL display canvas (Fl_Gl_Window).  It is held within
						a TrainWindow
						that is the outer window with all the widgets.
						The MyView needs
						to be aware of the window - since it might need to
						check the widgets to see how to draw

	  Note:        we need to have pointers to this, but maybe not know
						about it (beware circular references)

	 Platform:    Visio Studio 2019

*************************************************************************/

#include <iostream>
#include <Fl/fl.h>

// we will need OpenGL, and OpenGL needs windows.h
#include <windows.h>
//#include "GL/gl.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "GL/glu.h"

#include "MyView.h"
#include "MyWindow.h"
#include "Utilities/3DUtils.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;


//************************************************************************
//
// * Constructor to set up the GL window
//========================================================================
MyView::
MyView(int x, int y, int w, int h, const char* l)
	: Fl_Gl_Window(x, y, w, h, l)
	//========================================================================
{
	mode(FL_RGB | FL_ALPHA | FL_DOUBLE | FL_STENCIL);

	//Fl::add_idle(IdleCallback, this);

	resetArcball();
}

//************************************************************************
//
// * Reset the camera to look at the world
//========================================================================
void MyView::
resetArcball()
//========================================================================
{
	// Set up the camera to look at the world
	// these parameters might seem magical, and they kindof are
	// a little trial and error goes a long way
	arcball.setup(this, 40, 2.50, .2f, .4f, 0);
}

//************************************************************************
//
// * FlTk Event handler for the window
//########################################################################
// TODO: 
//       if you want to make the train respond to other events 
//       (like key presses), you might want to hack this.
//########################################################################
//========================================================================
int MyView::handle(int event)
{
	// see if the ArcBall will handle the event - if it does, 
	// then we're done
	// note: the arcball only gets the event if we're in world view
	if (mw->world_cam->value())
		if (arcball.handle(event))
			return 1;

	// remember what button was used
	static int last_push;

	switch (event) {
		// Mouse button being pushed event
	case FL_PUSH:
		last_push = Fl::event_button();
		// if the left button be pushed is left mouse button
		if (last_push == FL_LEFT_MOUSE) {
			doPick(Fl::event_x(), Fl::event_y());
			damage(1);
			return 1;
		};
		break;

		// Mouse button release event
	case FL_RELEASE: // button release
		damage(1);
		last_push = 0;
		return 1;

		// Mouse button drag event
	case FL_DRAG:
		break;

		// in order to get keyboard events, we need to accept focus
	case FL_FOCUS:
		return 1;

		// every time the mouse enters this window, aggressively take focus
	case FL_ENTER:
		focus(this);
		break;

	case FL_KEYBOARD:
		int k = Fl::event_key();
		int ks = Fl::event_state();
		if (k == 'p') {

			return 1;
		};
		break;
	}

	return Fl_Gl_Window::handle(event);
}

//************************************************************************
//
// * this is the code that actually draws the window
//   it puts a lot of the work into other routines to simplify things
//========================================================================
void MyView::draw()
{
	//*********************************************************************
	//
	// * Set up basic opengl informaiton
	//
	//**********************************************************************
	//initialized glad
	if (gladLoadGL())
	{
		//initiailize VAO, VBO, Shader...

		std::string common_lib = Shader::readCode("../MeshSimplification/src/shaders/common_lib.glsl");
		std::string material_lib = Shader::readCode("../MeshSimplification/src/shaders/material_lib.glsl");

		if (!this->commom_shader) {
			this->commom_shader = new Shader(
				common_lib + Shader::readCode("../MeshSimplification/src/shaders/simple.vert"),
				std::string(), std::string(), std::string(),
				Shader::readCode("../MeshSimplification/src/shaders/simple.frag"));
		}
		if (!this->commom_matrices) {
			this->commom_matrices = new UBO();
			this->commom_matrices->size = 3 * sizeof(glm::mat4);
			glGenBuffers(1, &this->commom_matrices->ubo);
			glBindBuffer(GL_UNIFORM_BUFFER, this->commom_matrices->ubo);
			glBufferData(GL_UNIFORM_BUFFER, this->commom_matrices->size, NULL, GL_STATIC_DRAW);
			glBindBuffer(GL_UNIFORM_BUFFER, 0);
		}

		if (!this->picking_shader) {
			this->picking_shader = new Shader(
				common_lib + Shader::readCode("../MeshSimplification/src/shaders/picking.vert"),
				std::string(), std::string(), std::string(),
				Shader::readCode("../MeshSimplification/src/shaders/picking.frag"));
			picking_tex.Init(w(), h());
		}

		if (!this->gl_mesh)
		{
			this->gl_mesh = new GLMesh();
			//this->gl_mesh->Init("D:/maochinn/NTUST/Course/DigitalMesh/project/build/output.obj");
		}
	}
	else
		throw std::runtime_error("Could not initialize GLAD!");


	// Set up the view port
	glViewport(0, 0, w(), h());

	// clear the window, be sure to clear the Z-Buffer too
	glClearColor(0.958, 0.909, 0.871, 0);		// background should be blue

	// we need to clear out the stencil buffer since we'll use
	// it for shadows
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();		// put the code to set up matrices here
	glEnable(GL_DEPTH_TEST);

	//
	setUBO();
	glBindBufferRange(GL_UNIFORM_BUFFER, /*binding point*/0, this->commom_matrices->ubo, 0, this->commom_matrices->size);

	// model position (0,0,0)
	glm::mat4 model_matrix = glm::mat4();

	//bind shader
	this->commom_shader->Use();

	glUniformMatrix4fv(glGetUniformLocation(this->commom_shader->Program, "u_model"), 1, GL_FALSE, &model_matrix[0][0]);
	glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(.941f, .25f, .25f)[0]);
	this->gl_mesh->renderMesh();
	glDisable(GL_DEPTH_TEST);

	glUniform3fv(glGetUniformLocation(this->commom_shader->Program, "u_color"), 1, &glm::vec3(.26f, .181f, .172f)[0]);
	glLineWidth(1.38f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	this->gl_mesh->renderMesh();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	//unbind shader(switch to fixed pipeline)
	glUseProgram(0);

	this->gl_mesh->renderControlPoints();

}

void MyView::resize(int x, int y, int w, int h)
{
	Fl_Gl_Window::resize(x, y, w, h);
	this->picking_tex.Resize(w, h);
}

void MyView::doPick(int mx, int my)
{
	// bind picking
	this->picking_shader->Use();
	this->picking_tex.EnableWriting();
	glViewport(0, 0, w(), h());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();		// put the code to set up matrices here
	glEnable(GL_DEPTH_TEST);

	//
	setUBO();
	glBindBufferRange(GL_UNIFORM_BUFFER, /*binding point*/0, this->commom_matrices->ubo, 0, this->commom_matrices->size);

	// model position (0,0,0)
	glm::mat4 model_matrix = glm::mat4();
	glUniformMatrix4fv(glGetUniformLocation(this->picking_shader->Program, "u_model"), 1, GL_FALSE, &model_matrix[0][0]);

	if (mw->renderMeshButton->value())
		this->gl_mesh->renderMesh();

	this->picking_tex.DisableWriting();

	glUseProgram(0);

	float PrimID = this->picking_tex.ReadPixel(mx, h() - my - 1);
	if (int(PrimID) > 0) {
		std::cout << PrimID << std::endl;

		// these positions must be in range [-1, 1] (!!!), not [0, width] and [0, height]
		float mouseX = mx / (w() * 0.5f) - 1.0f;
		float mouseY = my / (h() * 0.5f) - 1.0f;

		GLfloat modelViewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);

		GLfloat projectionViewMatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, projectionViewMatrix);

		glm::mat4 proj = glm::make_mat4(projectionViewMatrix);
		glm::mat4 view = glm::make_mat4(modelViewMatrix);

		glm::mat4 invVP = glm::inverse(proj * view);
		glm::vec4 screenPos = glm::vec4(mouseX, -mouseY, 1.0f, 1.0f);
		glm::vec4 worldPos = invVP * screenPos;

		std::cout << worldPos.x << " " << worldPos.y << " " << worldPos.z << " " << worldPos.w << std::endl;

		this->gl_mesh->select(int(PrimID), MyMesh::Point(worldPos.x, 0 , worldPos.z));
	}
	else {
		std::cout << "Nope" << std::endl;
	}
}

//************************************************************************
//
// * This sets up both the Projection and the ModelView matrices
//   HOWEVER: it doesn't clear the projection first (the caller handles
//   that) - its important for picking
//========================================================================
void MyView::
setProjection()
//========================================================================
{
	// Compute the aspect ratio (we'll need it)
	float aspect = static_cast<float>(w()) / static_cast<float>(h());

	// Check whether we use the world camp
	if (mw->world_cam->value())
		arcball.setProjection(false);
	// Or we use the top cam
	else if (mw->top_cam->value()) {
		float wi, he;
		if (aspect >= 1) {
			wi = 110;
			he = wi / aspect;
		}
		else {
			he = 110;
			wi = he * aspect;
		}

		// Set up the top camera drop mode to be orthogonal and set
		// up proper projection matrix
		glMatrixMode(GL_PROJECTION);
		glOrtho(-wi, wi, -he, he, 200, -200);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(-90, 1, 0, 0);
	}
}

void MyView::setUBO()
{
	float wdt = this->pixel_w();
	float hgt = this->pixel_h();

	glm::mat4 view_matrix;
	glGetFloatv(GL_MODELVIEW_MATRIX, &view_matrix[0][0]);
	//HMatrix view_matrix; 
	//this->arcball.getMatrix(view_matrix);

	glm::mat4 projection_matrix;
	glGetFloatv(GL_PROJECTION_MATRIX, &projection_matrix[0][0]);
	//projection_matrix = glm::perspective(glm::radians(this->arcball.getFoV()), (GLfloat)wdt / (GLfloat)hgt, 0.01f, 1000.0f);


	glBindBuffer(GL_UNIFORM_BUFFER, this->commom_matrices->ubo);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), &projection_matrix[0][0]);
	glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), &view_matrix[0][0]);
	glBufferSubData(GL_UNIFORM_BUFFER, 2 * sizeof(glm::mat4), sizeof(glm::mat4), &glm::inverse(view_matrix)[0][0]);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

bool PickingTexture::Init(unsigned int WindowWidth, unsigned int WindowHeight)
{
	// Create the FBO
	glGenFramebuffers(1, &m_fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

	// Create the texture object for the primitive information buffer
	glGenTextures(1, &m_pickingTexture);
	glBindTexture(GL_TEXTURE_2D, m_pickingTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, WindowWidth, WindowHeight,
		0, GL_RGB, GL_FLOAT, NULL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
		m_pickingTexture, 0);

	// Disable reading to avoid problems with older GPUs
	glReadBuffer(GL_NONE);

	glDrawBuffer(GL_COLOR_ATTACHMENT0);

	// Verify that the FBO is correct
	GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if (Status != GL_FRAMEBUFFER_COMPLETE) {
		printf("FB error, status: 0x%x\n", Status);
		return false;
	}

	// Restore the default framebuffer
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

bool PickingTexture::Resize(unsigned int WindowWidth, unsigned int WindowHeight)
{
	// Create the FBO
	glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

	// Create the texture object for the primitive information buffer
	glBindTexture(GL_TEXTURE_2D, m_pickingTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, WindowWidth, WindowHeight,
		0, GL_RGB, GL_FLOAT, NULL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
		m_pickingTexture, 0);

	// Disable reading to avoid problems with older GPUs
	glReadBuffer(GL_NONE);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);

	// Verify that the FBO is correct
	GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if (Status != GL_FRAMEBUFFER_COMPLETE) {
		printf("FB error, status: 0x%x\n", Status);
		return false;
	}

	// Restore the default framebuffer
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return true;
}

void PickingTexture::EnableWriting()
{
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
}

void PickingTexture::DisableWriting()
{
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

float PickingTexture::ReadPixel(unsigned int x, unsigned int y)
{
	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo);
	glReadBuffer(GL_COLOR_ATTACHMENT0);

	float Pixel;
	glReadPixels(x, y, 1, 1, GL_RGB, GL_FLOAT, &Pixel);

	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	return Pixel;
}