#include <map>
#include <set>
#include <algorithm>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"

//#include "../CDT/include/CDT.h"
//#include "../CDT/extras/VerifyTopology.h"

#define CGAL_MESH_2_OPTIMIZER_VERBOSE
//#define CGAL_MESH_2_OPTIMIZERS_DEBUG
//#define CGAL_MESH_2_SIZING_FIELD_USE_BARYCENTRIC_COORDINATES
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/lloyd_optimize_mesh_2.h>

#include "MeshObject.h"

#include <fstream>

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	assert(!(hi < lo));
	return (v < lo) ? lo : (hi < v) ? hi : v;
}

inline double cot(Eigen::Vector3d v, Eigen::Vector3d w) {
	if (v == w)
		return std::numeric_limits<double>::infinity();
	return(v.dot(w) / v.cross(w).norm());
};

#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
	request_halfedge_status();
}

MyMesh::~MyMesh()
{

}

bool MyMesh::reloadMesh(std::vector<MyMesh::Point> vertices, std::vector<unsigned int> indices)
{
	if (vertices.empty() || indices.empty())
		return false;

	this->ClearMesh();

	std::vector<MyMesh::VertexHandle> v_handles;
	for (OpenMesh::Vec3f v : vertices)
	{
		v_handles.push_back(this->add_vertex(v));
	}
	std::vector<MyMesh::VertexHandle>  face_vhandles;
	for (int i = 0; i < indices.size(); i += 3)
	{
		face_vhandles.clear();
		face_vhandles.push_back(v_handles[indices[i]]);
		face_vhandles.push_back(v_handles[indices[i + 1]]);
		face_vhandles.push_back(v_handles[indices[i + 2]]);
		this->add_face(face_vhandles);
	}

	this->update_normals();
}

int MyMesh::FindVertex(MyMesh::Point pointToFind)
{
	int idx = -1;
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		MyMesh::Point p = point(*v_it);
		if (pointToFind == p)
		{
			idx = v_it->idx();
			break;
		}
	}

	return idx;
}

void MyMesh::ClearMesh()
{
	if (!faces_empty())
	{
		for (MyMesh::FaceIter f_it = faces_begin(); f_it != faces_end(); ++f_it)
		{
			delete_face(*f_it, true);
		}

		garbage_collection();
	}
}

void MyMesh::Registration()
{
	this->add_property(prop_G, "prop_G");
	
	// precompute L1, L2 and G
	preComputeG();
	preComputeL1();
	preComputeL2();
}

void MyMesh::preComputeG()
{
	// pre G
	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {
		
		int rows = (is_boundary(e_it) ? 6 : 8);
		int row = 0;

		Eigen::MatrixXd G = Eigen::MatrixXd::Zero(rows, 2);

		HalfedgeHandle heh = halfedge_handle(e_it, 0);
		MyMesh::Point pFrom = point(from_vertex_handle(heh));
		G(row, 0) = pFrom[0];
		G(row, 1) = pFrom[1];
		row += 1;
		G(row, 0) = pFrom[1];
		G(row, 1) = -pFrom[0];
		row += 1;

		MyMesh::Point pTo = point(to_vertex_handle(heh));
		G(row, 0) = pTo[0];
		G(row, 1) = pTo[1];
		row += 1;
		G(row, 0) = pTo[1];
		G(row, 1) = -pTo[0];
		row += 1;

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p0 = point(vh0);
			G(row, 0) = p0[0];
			G(row, 1) = p0[1];
			row += 1;
			G(row, 0) = p0[1];
			G(row, 1) = -p0[0];
			row += 1;
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			MyMesh::Point p1 = point(vh1);
			G(row, 0) = p1[0];
			G(row, 1) = p1[1];
			row += 1;
			G(row, 0) = p1[1];
			G(row, 1) = -p1[0];
		}

		G = (G.transpose() * G).inverse() * G.transpose();
		this->property(prop_G, e_it) = G;
	}
}

void MyMesh::preComputeL1()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L1 = Eigen::SparseMatrix<double>(N_E * 2, N_V * 2);
	std::vector<Eigen::Triplet<double>> triplet_list_L1;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx() * 2;

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		// edge vector
		MyMesh::Point from_to = point(vh_to) - point(vh_from);
		Eigen::MatrixXd e(2, 2);
		e << from_to[0], from_to[1],
			from_to[1], -from_to[0];

		Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, cols);
		h(0, 0) = -1;
		h(0, 2) = 1;
		h(1, 1) = -1;
		h(1, 3) = 1;

		h = h - (e * G);

		int col = vh_from.idx() * 2;
		int hcol = 0;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col+1, h(0, hcol+1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row+1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row+1, col+1, h(1, hcol+1)));

		col = vh_to.idx() * 2;
		hcol += 2;
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
		triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));

		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			col = vh0.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}

		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			col = vh1.idx() * 2;
			hcol += 2;
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col, h(0, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row, col + 1, h(0, hcol + 1)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col, h(1, hcol)));
			triplet_list_L1.push_back(Eigen::Triplet<double>(row + 1, col + 1, h(1, hcol + 1)));
		}
	}

	L1.setFromTriplets(triplet_list_L1.begin(), triplet_list_L1.end());
	LL1 = L1.transpose() * L1;
}

void MyMesh::preComputeL2()
{
	const int N_E(n_edges());
	const int N_V(n_vertices());

	L2 = Eigen::SparseMatrix<double>(N_E, N_V);
	std::vector<Eigen::Triplet<double>> triplet_list_L2;

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_from.idx(), -1));
		triplet_list_L2.push_back(Eigen::Triplet<double>(row, vh_to.idx(), 1));
	}

	L2.setFromTriplets(triplet_list_L2.begin(), triplet_list_L2.end());
	LL2 = L2.transpose() * L2;
}

void MyMesh::select(unsigned int face_ID, MyMesh::Point p)
{
	FaceHandle fh = this->face_handle(face_ID-1);

	FaceVertexIter fv_it = fv_iter(fh);
	MyMesh::Point p0 = point(fv_it); ++fv_it;
	MyMesh::Point p1 = point(fv_it); ++fv_it;
	MyMesh::Point p2 = point(fv_it);

	MyMesh::Point vp0 = p0 - p;
	MyMesh::Point vp1 = p1 - p;
	MyMesh::Point vp2 = p2 - p;

	double a0 = vp1.cross(vp2).length();
	double a1 = vp0.cross(vp2).length();
	double a2 = vp0.cross(vp1).length();

	double i_area = 1.0 / (a0 + a1 + a2);

	ControlPoint cp;
	cp.fh = fh;
	cp.w[0] = i_area * a0;
	cp.w[1] = i_area * a1;
	cp.w[2] = i_area * a2;
	cp.c = cp.w[0] * p0 + cp.w[1] * p1 + cp.w[2] * p2;

	AddControlPoint(cp);
}

void MyMesh::InitCompilation()
{
	C1_triplets.clear();
	C2_triplets.clear();
}

void MyMesh::AddControlPoint(ControlPoint cp)
{
	FaceVertexIter fv_it = fv_iter(cp.fh);
	int c0 = fv_it->idx(); ++fv_it;
	int c1 = fv_it->idx(); ++fv_it;
	int c2 = fv_it->idx();

	int row = controlPoints.size();

	C1_triplets.push_back(Eigen::Triplet<double>(row * 2, c0 * 2, cp.w[0] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(row * 2, c1 * 2, cp.w[1] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(row * 2, c2 * 2, cp.w[2] * W));

	C1_triplets.push_back(Eigen::Triplet<double>(row * 2 + 1, c0 * 2 + 1, cp.w[0] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(row * 2 + 1, c1 * 2 + 1, cp.w[1] * W));
	C1_triplets.push_back(Eigen::Triplet<double>(row * 2 + 1, c2 * 2 + 1, cp.w[2] * W));

	C2_triplets.push_back(Eigen::Triplet<double>(row, c0, cp.w[0] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(row, c1, cp.w[1] * W));
	C2_triplets.push_back(Eigen::Triplet<double>(row, c2, cp.w[2] * W));

	controlPoints.push_back(cp);

	Compilation();
}

void MyMesh::RemoveControlPoint(unsigned int idx)
{
	ControlPoint& cp = controlPoints[idx];

	C1_triplets.erase(C1_triplets.begin() + idx * 6, C1_triplets.begin() + idx * 6 + 6);
	C2_triplets.erase(C2_triplets.begin() + idx * 3, C2_triplets.begin() + idx * 3 + 3);

	controlPoints.erase(controlPoints.begin() + idx);

	Compilation();
}

void MyMesh::Compilation()
{
	std::cout << 0 << std::endl;
	const int N_V(n_vertices());

	C1 = Eigen::SparseMatrix<double>(controlPoints.size() * 2, N_V * 2); // solve x,y together
	C2 = Eigen::SparseMatrix<double>(controlPoints.size(), N_V); // solve x, y respectively

	CC1 = Eigen::SparseMatrix<double>(N_V * 2, N_V * 2); // solve x,y together
	CC2 = Eigen::SparseMatrix<double>(N_V, N_V); // solve x, y respectively

	C1.setFromTriplets(C1_triplets.begin(), C1_triplets.end());
	C2.setFromTriplets(C2_triplets.begin(), C2_triplets.end());

	CC1 = C1.transpose() * C1;
	CC2 = C2.transpose() * C2;

	std::cout << 5 << std::endl;

}

unsigned int MyMesh::FindControlPoint(MyMesh::Point, double)
{
	return 0;
}


void MyMesh::Compute()
{
	Step1();
	Step2();
}

void MyMesh::Step1()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	Eigen::MatrixXd b1 = Eigen::MatrixXd::Zero(1, N_E * 2 + N_C * 2);

	for (int i = 0; i < controlPoints.size(); i++) {
		b1(0, N_E * 2 + i * 2) = controlPoints[i].c[0] * W;
		b1(0, N_E * 2 + i * 2 + 1) =  controlPoints[i].c[1] * W;
	}

	Eigen::SparseMatrix<double, Eigen::RowMajor> A1(N_E * 2 + N_C * 2, N_V * 2);
	A1.topRows(N_E * 2) = L1;
	A1.bottomRows(N_C * 2) = C1;

	Eigen::SparseMatrix<double> AA1 = LL1 + CC1;

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AA1);

	V1 = solver.solve(A1.transpose() * b1);
}

void MyMesh::Step2()
{
	const int N_V(n_vertices());
	const int N_E(n_edges());
	const int N_C(controlPoints.size());

	Eigen::MatrixXd b2x(1, N_E + N_C);
	Eigen::MatrixXd b2y(1, N_E + N_C);

	for (auto e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {

		int cols = (is_boundary(e_it) ? 6 : 8);
		int row = e_it->idx();

		HalfedgeHandle heh = halfedge_handle(e_it, 0);

		Eigen::MatrixXd& G = this->property(prop_G, e_it);

		VertexHandle vh_from = from_vertex_handle(heh);
		VertexHandle vh_to = to_vertex_handle(heh);

		double pFromX = V1(vh_from.idx() * 2);
		double pFromY = V1(vh_from.idx() * 2 + 1);

		double pToX = V1(vh_to.idx() * 2);
		double pToY = V1(vh_to.idx() * 2 + 1);

		double c = 0;
		double s = 0;

		Eigen::VectorXd v(cols);

		c += G(0, 0) * pFromX + G(0, 1) * pFromY;
		s += G(1, 0) * pFromX + G(1, 1) * pFromY;

		c += G(0, 2) * pToX + G(0, 3) * pToY;
		s += G(1, 2) * pToX + G(1, 3) * pToY;

		int v_row = 2;
		// boundary check
		VertexHandle vh0 = opposite_vh(heh);
		if (vh0 != MyMesh::InvalidVertexHandle) {
			double p0X = V1(vh0.idx() * 2);
			double p0Y = V1(vh0.idx() * 2 + 1);
			c += G(0, v_row * 2) * p0X + G(0, v_row * 2 + 1) * p0Y;
			s += G(1, v_row * 2) * p0X + G(1, v_row * 2 + 1) * p0Y;
			v_row += 1;
		}
		VertexHandle vh1 = opposite_he_opposite_vh(heh);
		if (vh1 != MyMesh::InvalidVertexHandle) {
			double p0X = V1(vh1.idx() * 2);
			double p0Y = V1(vh1.idx() * 2 + 1);
			c += G(0, v_row * 2) * p0X + G(0, v_row * 2 + 1) * p0Y;
			s += G(1, v_row * 2) * p0X + G(1, v_row * 2 + 1) * p0Y;
		}

		double norm = 1.0 / (c * c + s * s);
		MyMesh::Point e = point(vh_to) - point(vh_from);

		b2x(0, row) = (e[0] * c + e[1] * s) * norm;
		b2y(0, row) = (e[1] * c - e[0] * s) * norm;
	}

	for (int i = 0; i < controlPoints.size(); i++) {
		b2x(0, N_E + i) = controlPoints[i].c[0] * W;
		b2y(0, N_E + i) = controlPoints[i].c[1] * W;
	}

	Eigen::SparseMatrix<double, Eigen::RowMajor> A2(N_E + N_C, N_V);
	A2.topRows(N_E) = L2;
	A2.bottomRows(N_C) = C2;

	Eigen::SparseMatrix<double> AA2 = LL2 + CC2;

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AA2);

	V2x = solver.solve(A2.transpose() * b2x);
	V2y = solver.solve(A2.transpose() * b2y);
}

#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{
}

GLMesh::~GLMesh()
{

}

bool GLMesh::Init(std::string fileName)
{
	if (Load2DModel(fileName))
	{
		glGenVertexArrays(1, &this->vao.vao);
		glBindVertexArray(this->vao.vao);

		glGenBuffers(3, this->vao.vbo);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(1);

		glGenBuffers(1, &this->vao.ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		LoadToShader();

		std::cout << "SUCCESS" << std::endl;
		return true;
	}

	std::cout << "FAILED" << std::endl;
	return false;
}

void GLMesh::renderMesh()
{
	if (this->vao.element_amount > 0)
	{
		glBindVertexArray(this->vao.vao);
		glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
}
void GLMesh::renderControlPoints()
{
	//glEnable(GL_PROGRAM_POINT_SIZE);
	glColor3d(0, 0, 1);
	glPointSize(10);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	size_t n_controlPoints = this->mesh.controlPoints.size();
	for (size_t i = 0; i < n_controlPoints; i++) {
		MyMesh::ControlPoint& cp = this->mesh.controlPoints[i];
		glVertex3f(cp.c[0], 0, cp.c[2]);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);
	//glDisable(GL_PROGRAM_POINT_SIZE);
}

void GLMesh::select(unsigned int tri_ID, MyMesh::Point p)
{
	this->mesh.select(tri_ID, p);
}

bool GLMesh::LoadModel(std::string fileName)
{
	OpenMesh::IO::Options ropt;
	if (OpenMesh::IO::read_mesh(mesh, fileName, ropt))
	{
		if (!ropt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_vertex_normals())
		{
			mesh.request_face_normals();
			mesh.update_normals();
			//mesh.release_face_normals();
		}
		return true;
	}

	return false;
}

bool GLMesh::Load2DImage(std::string fileName)
{
	cv::Mat img = cv::imread(fileName);
	return false;
}


typedef CGAL::Exact_predicates_inexact_constructions_kernel           CGAL_K;
typedef CGAL::Delaunay_mesh_vertex_base_2<CGAL_K>                     CGAL_Vb;
typedef CGAL::Delaunay_mesh_face_base_2<CGAL_K>                       CGAL_Fb;
typedef CGAL::Triangulation_data_structure_2<CGAL_Vb, CGAL_Fb>        CGAL_Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL_K, CGAL_Tds>  CGAL_CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CGAL_CDT>                 CGAL_Criteria;
typedef CGAL::Delaunay_mesher_2<CGAL_CDT, CGAL_Criteria>              CGAL_Mesher;

typedef CGAL_CDT::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_CDT::Point CGAL_Point;

bool GLMesh::Load2DModel(std::string fileName)
{
	std::ifstream ifs(fileName);
	if (!ifs.is_open()) {
		std::cout << "Cannot open file \"" << fileName << "\" !!" << std::endl;
		return false;
	}

	std::size_t nPts, nEdges;
	ifs >> nPts >> nEdges;

	std::vector<MyMesh::Point> m_points;

	for (std::size_t i = 0; i < nPts; ++i)
	{
		float x1, y1;
		ifs >> x1 >> y1;
		m_points.push_back(MyMesh::Point(x1, y1, 0));
	}
	// find bounding box
	float max_x = m_points[0][0];
	float max_y = m_points[0][1];
	float min_x = m_points[0][0];
	float min_y = m_points[0][1];
	for (int i = 0; i < m_points.size(); i++)
	{
		max_x = std::max(m_points[i][0], max_x);
		max_y = std::max(m_points[i][1], max_y);
		min_x = std::min(m_points[i][0], min_x);
		min_y = std::min(m_points[i][1], min_y);
	}

	float norm_size = 1.0f;
	float norm_scale = norm_size / std::max(std::max(abs(max_x - min_x), abs(max_y - min_y)), 1.0f);
	float x_offset = (max_x + min_x) * norm_scale * 0.5f;
	float y_offset = (max_y + min_y) * norm_scale * 0.5f;

	// create constrainted delaunay triangulation handler
	CGAL_CDT cdt;

	// insertion
	std::vector<CGAL_Vertex_handle> vertices;
	for (int i = 0; i < m_points.size(); i++)
	{
		vertices.push_back(
			cdt.insert(CGAL_Point(m_points[i][0] * norm_scale - x_offset, m_points[i][1] * norm_scale - y_offset))
		);
	}

	for (std::size_t i = 0; i < nEdges; ++i)
	{
		unsigned int v1, v2;
		ifs >> v1 >> v2;
		cdt.insert_constraint(vertices[v1], vertices[v2]);
	}

	std::list<CGAL_Point> list_of_seeds;
	if (!ifs.eof()) {
		std::size_t nSeeds;
		ifs >> nSeeds;
		for (std::size_t i = 0; i < nSeeds; ++i)
		{
			float x1, y1;
			ifs >> x1 >> y1;
			list_of_seeds.push_back(CGAL_Point(x1 * norm_scale - x_offset, y1 * norm_scale - y_offset));
		}
	}

	ifs.close();

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;

	

	std::cout << "Meshing..." << std::endl;
	CGAL_Mesher mesher(cdt);
	mesher.set_criteria(CGAL_Criteria(0.125, 0.05));
	mesher.refine_mesh();

	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
	std::cout << "Run Lloyd optimization...";

	CGAL::lloyd_optimize_mesh_2(cdt, CGAL::parameters::max_iteration_number = 10);

	if (!list_of_seeds.empty()) {
		std::cout << "Meshing the domain..." << std::endl;
		CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
			CGAL_Criteria());
	}

	std::cout << " done." << std::endl;

	if (cdt.number_of_vertices() == 0)
		return false;
	
	float scale = 200.0f;
	std::map<CGAL_Vertex_handle,MyMesh::VertexHandle> v_handles;
	for (auto v_it = cdt.finite_vertices_begin(); v_it != cdt.finite_vertices_end(); ++v_it)
	{
		CGAL_Vertex_handle h = v_it->handle();
		auto& p = v_it->point();
		OpenMesh::Vec3f v(p.x() * scale, 0, p.y() * scale);
		v_handles[h] = mesh.add_vertex(v);
	}

	std::vector<MyMesh::VertexHandle> face_vhandles;
	for (auto f_it = cdt.finite_faces_begin(); f_it != cdt.finite_faces_end(); ++f_it)
	{
		if (f_it->is_in_domain()) {

			CGAL_Vertex_handle h0 = f_it->vertex(0)->handle();
			CGAL_Vertex_handle h1 = f_it->vertex(1)->handle();
			CGAL_Vertex_handle h2 = f_it->vertex(2)->handle();

			face_vhandles.clear();
			face_vhandles.push_back(v_handles[h0]);
			face_vhandles.push_back(v_handles[h1]);
			face_vhandles.push_back(v_handles[h2]);

			mesh.add_face(face_vhandles);
		}
	}

	return true;
}

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	std::vector<MyMesh::Normal> normals;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));
		normals.push_back(mesh.normal(*v_it));
	}
	std::vector<unsigned int> indices;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			indices.push_back(fv_it->idx());

	LoadToShader(vertices, normals, indices);
}
void GLMesh::LoadToShader(
	std::vector<MyMesh::Point>& vertices,
	std::vector<MyMesh::Normal>& normals,
	std::vector<unsigned int>& indices)
{
	this->vao.element_amount = indices.size();

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLMesh::LoadTexCoordToShader()
{
	if (mesh.has_vertex_texcoords2D())
	{
		std::vector<MyMesh::TexCoord2D> texCoords;
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			MyMesh::TexCoord2D texCoord = mesh.texcoord2D(*v_it);
			texCoords.push_back(texCoord);
		}

		glBindVertexArray(this->vao.vao);

		glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::TexCoord2D) * texCoords.size(), &texCoords[0], GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
}

void GLMesh::resetMesh()
{
	//mesh.reloadMesh(initial_vertices, initial_indices);
	LoadToShader();
}
bool GLMesh::exportMesh()
{
	// write mesh to output.obj
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output.obj"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			return false;
		}
	}
	catch (std::exception & x)
	{
		std::cerr << x.what() << std::endl;
		return false;
	}
	return true;
}

#pragma endregion
