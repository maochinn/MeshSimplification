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
	MyMesh();
	~MyMesh();

	bool reloadMesh(std::vector<MyMesh::Point>, std::vector<unsigned int>);

	int FindVertex(MyMesh::Point pointToFind);
	void ClearMesh();

	void Registration();
	void preComputeG();
	void preComputeL1();
	void preComputeL2();

	struct ControlPoint {
		MyMesh::FaceHandle fh;
		double w[3];
		MyMesh::Point c;
	};

	void select(unsigned int, MyMesh::Point);

	void InitCompilation();
	void AddControlPoint(ControlPoint);
	void RemoveControlPoint(unsigned int);
	void Compilation();

	unsigned int FindControlPoint(MyMesh::Point, double);

	void Compute();
	void Step1();
	void Step2();

	double W = 1000.0;

	std::vector<ControlPoint> controlPoints;

private:
	Eigen::SparseMatrix<double> L1, L2, LL1, LL2;
	Eigen::SparseMatrix<double> C1, C2, CC1, CC2;
	std::vector<Eigen::Triplet<double>> C1_triplets;
	std::vector<Eigen::Triplet<double>> C2_triplets;

	Eigen::VectorXd V1, V2x, V2y;

	OpenMesh::EPropHandleT<Eigen::MatrixXd> prop_G;
	//OpenMesh::EPropHandleT<float> prop_e;
	//OpenMesh::VPropHandleT<float> prop_sk_ve;
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

	void renderControlPoints();

	void select(unsigned int, MyMesh::Point);
private:
	MyMesh mesh;
	VAO vao;

	bool LoadModel(std::string fileName);
	bool Load2DImage(std::string fileName);
	bool Load2DModel(std::string fileName);
	void LoadToShader();
	void LoadToShader(
		std::vector<MyMesh::Point>& vertices,
		std::vector<MyMesh::Normal>& normals,
		std::vector<unsigned int>& indices);
	void LoadTexCoordToShader();
};

