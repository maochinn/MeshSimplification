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
#include "GL/glu.h"

#include "MyView.h"
#include "MyWindow.h"
#include "Utilities/3DUtils.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;


clock_t current_ticks, delta_ticks;
float fps = 0;

//void IdleCallback(void* pData)
//{
//	if (pData != NULL)
//	{
//		current_ticks = clock();
//
//		MyView* trainview = reinterpret_cast<MyView*>(pData);
//
//		delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
//		if (delta_ticks > 0)
//			fps = CLOCKS_PER_SEC / (float)delta_ticks;
//		//system("cls");
//		//std::cout << "FPS:" << fps << std::endl;
//	}
//}

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
	arcball.setup(this, 40, 250, .2f, .4f, 0);
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
			//doPick();
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

		if (!this->shader) {
			this->shader = new Shader(
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

		if (!this->gl_mesh)
		{
			this->gl_mesh = new GLMesh();
			this->gl_mesh->Init("../MeshSimplification/Models/neptune_200k_org.obj");
			//this->gl_mesh->Init("../MeshSimplification/Models/neptune_50k_hk.obj");

			//useless testing code
			{


				//PolyMesh mesh;
				//// Request required status flags
				//mesh.request_vertex_status();
				//mesh.request_edge_status();
				//mesh.request_halfedge_status();
				//mesh.request_face_status();
				//// Add some vertices as in the illustration above
				//PolyMesh::VertexHandle vhandle[7];
				//vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, 1, 0));
				//vhandle[1] = mesh.add_vertex(MyMesh::Point(-1, 3, 0));
				//vhandle[2] = mesh.add_vertex(MyMesh::Point(0, 0, 0));
				//vhandle[3] = mesh.add_vertex(MyMesh::Point(0, 2, 0));
				//vhandle[4] = mesh.add_vertex(MyMesh::Point(0, 4, 0));
				//vhandle[5] = mesh.add_vertex(MyMesh::Point(1, 1, 0));
				//vhandle[6] = mesh.add_vertex(MyMesh::Point(1, 3, 0));
				//// Add three quad faces
				//std::vector<PolyMesh::VertexHandle> face_vhandles;
				//face_vhandles.push_back(vhandle[1]);
				//face_vhandles.push_back(vhandle[0]);
				//face_vhandles.push_back(vhandle[2]);
				//face_vhandles.push_back(vhandle[3]);
				//mesh.add_face(face_vhandles);
				//face_vhandles.clear();
				//face_vhandles.push_back(vhandle[1]);
				//face_vhandles.push_back(vhandle[3]);
				//face_vhandles.push_back(vhandle[6]);
				//face_vhandles.push_back(vhandle[4]);
				//mesh.add_face(face_vhandles);
				//face_vhandles.clear();
				//face_vhandles.push_back(vhandle[3]);
				//face_vhandles.push_back(vhandle[2]);
				//face_vhandles.push_back(vhandle[5]);
				//face_vhandles.push_back(vhandle[6]);
				//mesh.add_face(face_vhandles);
				//// Now find the edge between vertex vhandle[2]
				//// and vhandle[3]


				////PolyMesh::HalfedgeHandle t23 = mesh.find_halfedge(vhandle[2], vhandle[3]);

				////for (PolyMesh::FaceIter it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
				////	std::cout << *it << std::endl;
				////	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(it); fv_it.is_valid(); ++fv_it) {
				////		std::cout << mesh.point(fv_it) << std::endl;
				////	}
				////}
				////puts("");
				////std::vector<PolyMesh::FaceHandle> vfs;
				////for (PolyMesh::VertexFaceIter vf_it = mesh.vf_iter(vhandle[3]); vf_it.is_valid(); ++vf_it) {
				////	std::cout << *vf_it << std::endl;
				////	vfs.push_back(vf_it.handle());
				////}
				////for (PolyMesh::FaceHandle vf :vfs) {
				////	puts("face");
				////	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(vf); fv_it.is_valid(); ++fv_it) {
				////		std::cout << mesh.point(fv_it) << std::endl;
				////	}
				////}

				////
				//std::vector<std::vector<PolyMesh::VertexHandle>> faces;
				//std::vector<PolyMesh::FaceHandle> temps;
				//for (PolyMesh::VertexFaceIter vf_it = mesh.vf_iter(vhandle[3]); vf_it.is_valid(); ++vf_it) {
				//	std::vector<PolyMesh::VertexHandle> vs;
				//	temps.push_back(vf_it.handle());
				//	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(vf_it); fv_it.is_valid(); ++fv_it) {
				//		vs.push_back(fv_it.handle());
				//	}
				//	faces.push_back(vs);
				//}
				//for (PolyMesh::VertexFaceIter vf_it = mesh.vf_iter(vhandle[2]); vf_it.is_valid(); ++vf_it) {
				//	auto it = std::find(temps.begin(), temps.end(), vf_it.handle());
				//	if (it == temps.end())
				//	{
				//		std::vector<PolyMesh::VertexHandle> vs;
				//		for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(vf_it); fv_it.is_valid(); ++fv_it) {
				//			vs.push_back(fv_it.handle());
				//		}
				//		faces.push_back(vs);
				//	}
				//}
				//for (auto verts : faces)
				//{
				//	puts("Face");
				//	for (auto vert : verts)
				//	{
				//		std::cout << mesh.point(vert) << std::endl;
				//	}
				//}

				//std::cout << mesh.n_vertices() << std::endl;
				//std::cout << mesh.n_edges() << std::endl;
				//std::cout << mesh.n_halfedges() << std::endl;
				//std::cout << mesh.n_faces() << std::endl;

				//for (PolyMesh::HalfedgeIter it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it) {
				//	if (mesh.to_vertex_handle(*it) == vhandle[3] &&
				//		mesh.from_vertex_handle(*it) == vhandle[2])
				//	{
				//		// Collapse edge
				//		mesh.collapse(*it);
				//		break;
				//	}
				//}

				////delete
				//std::vector<PolyMesh::FaceHandle> delete_faces;
				//
				//for (PolyMesh::VertexFaceIter vf_it = mesh.vf_iter(vhandle[3]); vf_it.is_valid(); ++vf_it) {
				//	delete_faces.push_back(vf_it);
				//}
				//for (auto df : delete_faces) {
				//	puts("face");
				//	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(df); fv_it.is_valid(); ++fv_it) {
				//		std::cout << mesh.point(fv_it) << std::endl;
				//	}
				//	mesh.delete_face(df, false);
				//}

				////re-add
				////mesh.add_vertex(mesh.point(vhandle[2]));
				//bool temp = mesh.status(vhandle[2]).deleted();
				//mesh.status(vhandle[2]).set_deleted(false);
				//temp = mesh.status(vhandle[2]).deleted();
				//for (auto verts : faces)
				//{
				//	puts("Face");
				//	std::vector<PolyMesh::VertexHandle> face;
				//	for (auto vert : verts)
				//	{
				//		face.push_back(vert);	
				//		std::cout << mesh.point(vert) << std::endl;
				//	}
				//	mesh.add_face(face);
				//}
				//mesh.garbage_collection();
				//std::cout << mesh.n_vertices() << std::endl;
				//std::cout << mesh.n_edges() << std::endl;
				//std::cout << mesh.n_halfedges() << std::endl;
				//std::cout << mesh.n_faces() << std::endl;

				////for (PolyMesh::VertexIter it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
				////{
				////	std::cout << mesh.point(it) << std::endl;
				////}

				//for (PolyMesh::FaceIter it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
				//	puts("face");
				//	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(it); fv_it.is_valid(); ++fv_it) {
				//		std::cout << mesh.point(fv_it) << std::endl;
				//	}
				//}
				//puts("");
				///*for (PolyMesh::FaceHandle vf : vfs) {
				//	puts("face");
				//	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(vf); fv_it.is_valid(); ++fv_it) {
				//		std::cout << mesh.point(fv_it) << std::endl;
				//	}
				//}*/

				////mesh.garbage_collection();
				////std::cout << mesh.n_vertices() << std::endl;
				////std::cout << mesh.n_edges() << std::endl;
				////std::cout << mesh.n_halfedges() << std::endl;
				////std::cout << mesh.n_faces() << std::endl;

				////PolyMesh::HalfedgeHandle t53 = mesh.find_halfedge(vhandle[5], vhandle[3]);
				////PolyMesh::HalfedgeHandle t56 = mesh.find_halfedge(vhandle[5], vhandle[6]);
				////PolyMesh::HalfedgeHandle t65 = mesh.find_halfedge(vhandle[6], vhandle[5]);
				////PolyMesh::FaceHandle f;

				////for (PolyMesh::FaceIter it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
				////	std::cout << *it << std::endl;
				////	for (PolyMesh::FaceVertexIter fv_it = mesh.fv_iter(it); fv_it.is_valid(); ++fv_it) {
				////		std::cout << mesh.point(fv_it) << std::endl;
				////	}

				//	//for (PolyMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(it); fh_it.is_valid(); ++fh_it) {
				//	//	if (fh_it.handle() == t56 || fh_it.handle() == t65)
				//	//	{
				//	//		f = it.handle();
				//	//		std::cout << "Halfedge has handle " << *fh_it << std::endl;
				//	//	}

				//	//}
				////}
				//
				////bool temp;
				////temp = mesh.status(t23).deleted();
				////temp = mesh.status(t53).deleted();
				////temp = mesh.status(t56).deleted();
				////temp = mesh.status(t65).deleted();
				////temp = mesh.status(f).deleted();
				////for (PolyMesh::HalfedgeIter it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it) {
				////	if (mesh.to_vertex_handle(*it) == vhandle[3] &&
				////		mesh.from_vertex_handle(*it) == vhandle[5])
				////	{
				////		// Collapse edge
				////		mesh.collapse(*it);
				////		break;
				////	}
				////}
				////temp = mesh.status(t53).deleted();
				////temp = mesh.status(t56).deleted();
				////temp = mesh.status(t65).deleted();
				////temp = mesh.status(f).deleted();
				////mesh.garbage_collection();
				////std::cout << mesh.n_vertices() << std::endl;
				////std::cout << mesh.n_edges() << std::endl;
				////std::cout << mesh.n_halfedges() << std::endl;
				////std::cout << mesh.n_faces() << std::endl;
			}
		}

		if (!this->plane) {
			GLfloat  vertices[] = {
				-0.5f ,0.0f , -0.5f,
				-0.5f ,0.0f , 0.5f ,
				0.5f ,0.0f ,0.5f ,
				0.5f ,0.0f ,-0.5f };
			GLfloat  normal[] = {
				0.0f, 1.0f, 0.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 1.0f, 0.0f };
			GLfloat  texture_coordinate[] = {
				0.0f, 0.0f,
				1.0f, 0.0f,
				1.0f, 1.0f,
				0.0f, 1.0f };
			GLuint element[] = {
				0, 1, 2,
				0, 2, 3, };

			this->plane = new VAO;
			this->plane->element_amount = sizeof(element) / sizeof(GLuint);
			glGenVertexArrays(1, &this->plane->vao);
			glGenBuffers(3, this->plane->vbo);
			glGenBuffers(1, &this->plane->ebo);

			glBindVertexArray(this->plane->vao);

			// Position attribute
			glBindBuffer(GL_ARRAY_BUFFER, this->plane->vbo[0]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
			glEnableVertexAttribArray(0);

			// Normal attribute
			glBindBuffer(GL_ARRAY_BUFFER, this->plane->vbo[1]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(normal), normal, GL_STATIC_DRAW);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
			glEnableVertexAttribArray(1);

			// Texture Coordinate attribute
			glBindBuffer(GL_ARRAY_BUFFER, this->plane->vbo[2]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(texture_coordinate), texture_coordinate, GL_STATIC_DRAW);
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (GLvoid*)0);
			glEnableVertexAttribArray(2);

			//Element attribute
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->plane->ebo);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(element), element, GL_STATIC_DRAW);

			// Unbind VAO
			glBindVertexArray(0);
		}

		if (!this->texture)
			this->texture = new Texture2D("../MeshSimplification/Images/church.png");

		if (!this->skybox) {
			this->skybox = new CubeMap(
				Shader(
					common_lib + Shader::readCode("../MeshSimplification/src/shaders/cubeMap.vert"),
					std::string(), std::string(), std::string(),
					material_lib + Shader::readCode("../MeshSimplification/src/shaders/cubeMap.frag")),
				"../MeshSimplification/Images/skybox/right.jpg",
				"../MeshSimplification/Images/skybox/left.jpg",
				"../MeshSimplification/Images/skybox/top.jpg",
				"../MeshSimplification/Images/skybox/bottom.jpg",
				"../MeshSimplification/Images/skybox/back.jpg",
				"../MeshSimplification/Images/skybox/front.jpg");
		}
	}
	else
		throw std::runtime_error("Could not initialize GLAD!");

	// Set up the view port
	glViewport(0, 0, w(), h());

	// clear the window, be sure to clear the Z-Buffer too
	glClearColor(0, 0, .3f, 0);		// background should be blue

	// we need to clear out the stencil buffer since we'll use
	// it for shadows
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH);

	// Blayne prefers GL_DIFFUSE
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();		// put the code to set up matrices here

	//######################################################################
	// TODO: 
	// you might want to set the lighting up differently. if you do, 
	// we need to set up the lights AFTER setting up the projection
	//######################################################################
	// enable the lighting
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// top view only needs one light
	if (mw->top_cam->value()) {
		glDisable(GL_LIGHT1);
		glDisable(GL_LIGHT2);
	}
	else {
		glEnable(GL_LIGHT1);
		glEnable(GL_LIGHT2);
	}

	//*********************************************************************
	//
	// * set the light parameters
	//
	//**********************************************************************
	GLfloat lightPosition1[] = { 0,1,1,0 }; // {50, 200.0, 50, 1.0};
	GLfloat lightPosition2[] = { 1, 0, 0, 0 };
	GLfloat lightPosition3[] = { 0, -1, 0, 0 };
	GLfloat yellowLight[] = { 0.5f, 0.5f, .1f, 1.0 };
	GLfloat whiteLight[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	GLfloat blueLight[] = { .1f,.1f,.3f,1.0 };
	GLfloat grayLight[] = { .3f, .3f, .3f, 1.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition1);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteLight);
	glLightfv(GL_LIGHT0, GL_AMBIENT, grayLight);

	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition2);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, yellowLight);

	glLightfv(GL_LIGHT2, GL_POSITION, lightPosition3);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, blueLight);



	//*********************************************************************
	// now draw the ground plane
	//*********************************************************************
	// set to opengl fixed pipeline(use opengl 1.x draw function)
	glUseProgram(0);

	setupFloor();
	glDisable(GL_LIGHTING);
	//drawFloor(200,10);


	//*********************************************************************
	// now draw the object and we need to do it twice
	// once for real, and then once for shadows
	//*********************************************************************
	glEnable(GL_LIGHTING);

	setUBO();
	glBindBufferRange(GL_UNIFORM_BUFFER, /*binding point*/0, this->commom_matrices->ubo, 0, this->commom_matrices->size);


	//bind shader
	this->shader->Use();

	glm::mat4 model_matrix = glm::mat4();
	model_matrix = glm::scale(model_matrix, glm::vec3(0.1f, 0.1f, 0.1f));
	model_matrix = glm::translate(model_matrix, glm::vec3(98.5175, 250.207, 1045.73));
	glUniformMatrix4fv(glGetUniformLocation(this->shader->Program, "u_model"), 1, GL_FALSE, &model_matrix[0][0]);
	glUniform3fv(glGetUniformLocation(this->shader->Program, "u_color"), 1, &glm::vec3(0.0f, 1.0f, 0.0f)[0]);
	this->texture->bind(0);
	//this->water->bindWaterTexture(0);
	//this->pool->bindCausticTexture(0);
	glUniform1i(glGetUniformLocation(this->shader->Program, "u_texture"), 0);

	////bind VAO
	//glBindVertexArray(this->plane->vao);
	////glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glDrawElements(GL_TRIANGLES, this->plane->element_amount, GL_UNSIGNED_INT, 0);
	////glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	////unbind VAO
	//glBindVertexArray(0);

	glBindVertexArray(this->gl_mesh->vao.vao);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	this->gl_mesh->Render();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//unbind VAO
	glBindVertexArray(0);


	this->skybox->render();

	//unbind shader(switch to fixed pipeline)
	glUseProgram(0);
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