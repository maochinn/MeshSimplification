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

	std::vector<std::vector<unsigned int>> degeneration_indices;

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
	void degenerateLeastSquareMesh(double, double, double S_L = 5.0, int iterations = 20);

	// Skeleton extraction sec 5
	struct SKHalfedge {
		VertexHandle from;
		VertexHandle to;
		float cost;
		SKHalfedge(VertexHandle, VertexHandle, float c = 0);
	};
	struct SKFace {
		VertexHandle from;
		VertexHandle to[2];
	};
	void degenerationMeshToLine();
	bool collapseToLine(std::vector<VertexHandle>& ,std::map<VertexHandle, std::vector<SKHalfedge>>&, std::map<VertexHandle, std::vector<SKFace>>& of_map);
	void initSKVertexErrorQuadric(std::map<VertexHandle, std::vector<SKHalfedge>>&, MyMesh::VertexHandle);
	void computeSKVertexError(std::map<VertexHandle, std::vector<SKHalfedge>>& ,MyMesh::VertexHandle);
	void computeSKEdgeCost(std::vector<SKHalfedge>::iterator);
	bool edge_is_collapse_ok(std::map<VertexHandle, std::vector<SKFace>>& of_map, std::map<VertexHandle, std::vector<SKHalfedge>>& ,std::vector<SKHalfedge>::iterator);
	bool edge_collapse(std::map<VertexHandle, std::vector<SKFace>>& of_map, std::map<VertexHandle, std::vector<SKHalfedge>>&, std::vector<SKHalfedge>::iterator);
	

private:

	float inv_avg_length;

	float last_min;
	bool use_last;
	MyMesh::EdgeHandle last_handle;

	OpenMesh::VPropHandleT<Eigen::Matrix4d> prop_Q;
	OpenMesh::EPropHandleT<MyMesh::Point> prop_v;
	OpenMesh::EPropHandleT<float> prop_e;

	OpenMesh::VPropHandleT<float> prop_sk_ve;
	OpenMesh::VPropHandleT<float> prop_sk_vl;
	OpenMesh::VPropHandleT<Eigen::Matrix4d> prop_sk_vQ;
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
	void degenerationMeshToLineSlider(float);
	void degenerationMeshToLine(float);

	int now_record_idx = 0;

	int render_mode = 0;
	
private:
	MyMesh mesh;
	MyMesh deg_simp_mesh;
	VAO vao;

	bool LoadModel(std::string fileName);
	void LoadToShader(std::vector<MyMesh::Point>&, std::vector<MyMesh::Normal>&, std::vector<unsigned int>&, int mode = 0);
};

