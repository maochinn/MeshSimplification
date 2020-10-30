#pragma once

#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <glad\glad.h>
#include <glm/glm.hpp>

#include <queue>

# include "BufferObject.h"

typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

class MyMesh : public TriMesh
{
public:
	std::vector<std::vector<MyMesh::Point>> record_vertices;
	std::vector<std::vector<MyMesh::Normal>> record_normals;
	std::vector<std::vector<unsigned int>> record_indices;

	MyMesh();
	~MyMesh();

	int FindVertex(MyMesh::Point pointToFind);
	void ClearMesh();

	static glm::mat4 computeErrorQuadrics(MyMesh::Point, MyMesh::Normal);
	void computeErrorQuadrics();

	void computeError(MyMesh::EdgeHandle);
	void computeError();
	
	void simplification();

	void record();

	bool collapse();
private:
	OpenMesh::VPropHandleT<glm::mat4> Q;
	OpenMesh::EPropHandleT<glm::mat4> new_Q;
	OpenMesh::EPropHandleT<MyMesh::Point> new_v;
	OpenMesh::EPropHandleT<float> epsilon;
};

class GLMesh
{
public:
	GLMesh();
	~GLMesh();

	bool Init(std::string fileName);
	void Render();
	void LoadTexCoordToShader();

	void simplification(float);

	void collispe();
	void uncollispe();

	int now_record_idx = 0;

	MyMesh mesh;
	VAO vao;
	//GLuint vao;
	//GLuint ebo;
	//GLuint vboVertices, vboNormal, vboTexCoord;

	

private:

	bool LoadModel(std::string fileName);
	void LoadToShader();
};

