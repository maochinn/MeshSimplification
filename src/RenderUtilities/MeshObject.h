#pragma once

#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <glad/glad.h>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Sparse>

# include "BufferObject.h"

typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

class MyMesh : public TriMesh
{
public:
	std::vector<std::vector<MyMesh::Point>> record_vertices;
	std::vector<std::vector<MyMesh::Normal>> record_normals;
	std::vector<std::vector<unsigned int>> record_indices;

	std::vector<std::vector<MyMesh::Point>> degeneration_vertices;

	MyMesh();
	~MyMesh();

	int FindVertex(MyMesh::Point pointToFind);
	void ClearMesh();

	// Simplification
	void computeErrorQuadrics(MyMesh::VertexHandle);
	void computeErrorQuadrics();

	void computeError(MyMesh::EdgeHandle);
	void computeError();
	
	void simplification();
	void record();
	bool collapse();

	// Least Square with random control points
	void generateLeastSquareMesh(std::vector<MyMesh::Point>&, int);

	// Skeleton extraction
	double computeWeight(MyMesh::HalfedgeHandle&);
	void degenerateLeastSquareMesh(std::vector<MyMesh::Point>&, double, double, double S_L = 5.0);
private:

	float last_min;
	bool use_last;
	MyMesh::EdgeHandle last_handle;

	OpenMesh::VPropHandleT<Eigen::Matrix4d> prop_Q;
	OpenMesh::EPropHandleT<MyMesh::Point> prop_v;
	OpenMesh::EPropHandleT<float> prop_e;
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

	bool exportSimplificationMesh(float);

	void generateLeastSquareMesh(int);

	void degenerateLeastSquareMesh(float);

	int now_record_idx = 0;
	
private:
	MyMesh mesh;
	VAO vao;

	bool LoadModel(std::string fileName);
	void LoadToShader(std::vector<MyMesh::Point>&, std::vector<MyMesh::Normal>&, std::vector<unsigned int>&);
};

