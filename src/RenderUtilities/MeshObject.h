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
	//std::vector<std::vector<MyMesh::Point>> record_vertices;
	//std::vector<std::vector<MyMesh::Normal>> record_normals;
	//std::vector<std::vector<unsigned int>> record_indices;

	//std::vector<std::vector<MyMesh::Point>> degeneration_vertices;

	//std::vector<std::vector<unsigned int>> degeneration_indices;

	MyMesh();
	~MyMesh();

	bool reloadMesh(std::vector<MyMesh::Point>, std::vector<unsigned int>);

	int FindVertex(MyMesh::Point pointToFind);
	void ClearMesh();

	// Simplification
	void computeErrorQuadrics(MyMesh::VertexHandle);
	void computeErrorQuadrics();

	void computeError(MyMesh::EdgeHandle);
	void computeError();

	bool collapse();
	void simplification(
		std::vector<std::vector<MyMesh::Point>>&,
		std::vector<std::vector<MyMesh::Normal>>&,
		std::vector<std::vector<unsigned int>>&);
	//record simplification's info
	void recordSimplification(
		std::vector<std::vector<MyMesh::Point>>&,
		std::vector<std::vector<MyMesh::Normal>>&,
		std::vector<std::vector<unsigned int>>&);

	// Least Square with random control points
	void generateLeastSquareMesh(std::vector<MyMesh::Point>&, int);

	// Skeleton extraction
	double computeLaplacianWeight(MyMesh::HalfedgeHandle&);
	void degenerateLeastSquareMesh(std::vector<std::vector<MyMesh::Point>>&, double, double, double S_L = 5.0, int iterations = 20);

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
	void degenerationMeshToLine(std::vector<std::vector<unsigned int>>& , std::vector<MyMesh::Point>&, double, double);
	bool collapseToLine(std::vector<VertexHandle>&,
		std::map<VertexHandle, std::vector<SKHalfedge>>&, std::map<VertexHandle, std::vector<SKFace>>&,
		std::map < VertexHandle, std::vector<VertexHandle>>&);
	void initSKVertexErrorQuadric(std::map<VertexHandle, std::vector<SKHalfedge>>&, MyMesh::VertexHandle);
	void computeSKVertexError(std::map<VertexHandle, std::vector<SKHalfedge>>&, MyMesh::VertexHandle);
	void computeSKEdgeCost(std::vector<SKHalfedge>::iterator);
	bool edge_is_collapse_ok(std::map<VertexHandle, std::vector<SKFace>>& of_map, std::map<VertexHandle, std::vector<SKHalfedge>>&, std::vector<SKHalfedge>::iterator);
	bool edge_collapse(std::map<VertexHandle, std::vector<SKFace>>& of_map, std::map<VertexHandle, std::vector<SKHalfedge>>&, std::vector<SKHalfedge>::iterator);


private:
	double SK_WA;
	double SK_WB;

	int sk_face_count;
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

	void resetMesh();
	bool exportMesh();

	void renderMesh();
	void renderSkeleton();

	void simplification();
	void simplification(float);

	void generateLeastSquareMesh(int);

	void degenerateLeastSquareMesh(double W0_L, double W0_H, double S_L);
	void degenerateLeastSquareMesh(float);
	void degenerationMeshToLine(double,double);
	void degenerationMeshToLine(float);

private:
	std::vector<MyMesh::Point> initial_vertices;
	std::vector<MyMesh::Normal> initial_normals;
	std::vector<unsigned int> initial_indices;

	std::vector<std::vector<MyMesh::Point>> simplification_vertices;
	std::vector<std::vector<MyMesh::Normal>> simplification_normals;
	std::vector<std::vector<unsigned int>> simplification_indices;

	std::vector<std::vector<MyMesh::Point>> degeneration_vertices;
	std::vector<unsigned int> degeneration_indices;

	std::vector<MyMesh::Point> skeleton_vertices;
	std::vector<MyMesh::Normal> skeleton_normal;
	std::vector<std::vector<unsigned int>> skeleton_indices;

	MyMesh mesh;
	VAO vao, skeleton;

	bool LoadModel(std::string fileName);
	void LoadToShader();
	void LoadToShader(
		std::vector<MyMesh::Point>& vertices,
		std::vector<MyMesh::Normal>& normals,
		std::vector<unsigned int>& indices);
	void LoadTexCoordToShader();
};

