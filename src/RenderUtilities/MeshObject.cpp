#include <map>
#include <set>
#include <algorithm>
#include "MeshObject.h"
struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	assert(!(hi < lo));
	return (v < lo) ? lo : (hi < v) ? hi : v;
}


#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
	//request_halfedge_status();
}

MyMesh::~MyMesh()
{

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
void MyMesh::computeErrorQuadrics(MyMesh::VertexHandle v_h)
{
	Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
	MyMesh::Point point = this->point(v_h);

	for (MyMesh::VertexFaceIter vf_it = this->vf_iter(v_h); vf_it.is_valid(); ++vf_it) {
		MyMesh::Normal normal = this->normal(vf_it);

		float a = normal[0];
		float b = normal[1];
		float c = normal[2];
		float d = -(a * point[0] + b * point[1] + c * point[2]);

		Eigen::Matrix4d K;
		K << a * a, a* b, a* c, a* d, \
			a* b, b* b, b* c, b* d, \
			a* c, b* c, c* c, c* d, \
			a* d, b* d, c* d, d* d;

		Q += K;
	}
	this->property(prop_Q, v_h) = Q;
}

void MyMesh::computeErrorQuadrics()
{
	this->add_property(prop_Q, "prop_Q");

	for (MyMesh::VertexIter v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it)
	{
		this->computeErrorQuadrics(v_it);
	}
}

void MyMesh::computeError(MyMesh::EdgeHandle e_handle)
{
	Eigen::Matrix4d& Q1 = this->property(prop_Q, to_vertex_handle(halfedge_handle(e_handle, 0)));
	Eigen::Matrix4d& Q2 = this->property(prop_Q, from_vertex_handle(halfedge_handle(e_handle, 0)));

	Eigen::Matrix4d Q = Q1 + Q2;

	Eigen::Matrix4d dQ(Q);
	dQ(3, 0) = 0;
	dQ(3, 1) = 0;
	dQ(3, 2) = 0;
	dQ(3, 3) = 1;
	double det = dQ.determinant();

	Eigen::Vector4d V(0, 0, 0, 1);
	float e = 0;

	MyMesh::Point& p1 = this->point(to_vertex_handle(halfedge_handle(e_handle, 0)));
	MyMesh::Point& p2 = this->point(from_vertex_handle(halfedge_handle(e_handle, 0)));

	if (abs(det) < 0.01f)
	{


		Eigen::Vector4d V0((p1[0] + p2[0]) * 0.5f, (p1[1] + p2[1]) * 0.5f, (p1[2] + p2[2]) * 0.5f, 1);
		Eigen::Vector4d V1(p1[0], p1[1], p1[2], 1);
		Eigen::Vector4d V2(p2[0], p2[1], p2[2], 1);

		float e0 = abs(V0.dot(Q * V0));
		float e1 = abs(V1.dot(Q * V1));
		float e2 = abs(V2.dot(Q * V2));

		if (e0 > e1 && e0 > e2) {
			V = V0;
			e = e0;
		}
		else if (e1 > e2) {
			V = V1;
			e = e1;
		}
		else {
			V = V2;
			e = e2;
		}
	}
	else
	{
		dQ = dQ.inverse().eval();
		V << dQ(0, 3), dQ(1, 3), dQ(2, 3), 1;
		e = abs(V.dot(Q * V));
	}

	this->property(prop_v, e_handle) = MyMesh::Point(V.x(), V.y(), V.z());
	this->property(prop_e, e_handle) = e;
}

void MyMesh::computeError()
{
	this->add_property(prop_v, "prop_v");
	this->add_property(prop_e, "prop_e"); 

	for (MyMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
	{
		computeError(e_it.handle());
	}
}
bool MyMesh::collapse()
{
	EdgeHandle e_handle;
	float min = FLT_MAX;

	bool found = false;

	if (use_last) {
		found = true;
		e_handle = last_handle;
		min = last_min;
	}
	else {
		for (MyMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
		{
			if (!status(e_it).deleted()) {
				float e = this->property(prop_e, e_it);
				if (e < min)
				{
					HalfedgeHandle tmp_he_h = halfedge_handle(e_it, 0);
					VertexHandle tmp_to = to_vertex_handle(tmp_he_h);

					MyMesh::Point old_V = this->point(tmp_to);

					MyMesh::Point tmp_V = this->property(prop_v, e_it);

					set_point(tmp_to, tmp_V);
					bool collapse_ok = is_collapse_ok(tmp_he_h);
					set_point(tmp_to, old_V);

					if (collapse_ok) {
						found = true;
						min = e;
						e_handle = e_it;

						if (min <= this->last_min) {
							break;
						}
					}
				}
			}
		}
	}

	if (!found) {
		std::cout << "Break" << std::endl;
		return false;
	}

	last_min = min;
	use_last = false;

	//print error
	//std::cout << min << std::endl;

	HalfedgeHandle he_handle = halfedge_handle(e_handle, 0);
	VertexHandle from = from_vertex_handle(he_handle);
	VertexHandle to = to_vertex_handle(he_handle);

	MyMesh::Point V = this->property(prop_v, e_handle);

	set_point(to, V);

	this->TriMesh::collapse(he_handle);

	//garbage_collection();

	//recompute quadrics & edge errors
	std::set<MyMesh::EdgeHandle> recalc_edges;

	this->property(prop_Q, to) = this->property(prop_Q, to) + this->property(prop_Q, from);

	for (MyMesh::VertexVertexIter vv_it = vv_iter(to); vv_it.is_valid(); ++vv_it)
	{
		if (!status(vv_it).deleted()) {
			for (MyMesh::VertexEdgeIter ve_it = ve_iter(vv_it); ve_it.is_valid(); ++ve_it)
			{
				if (!status(ve_it).deleted()) {
					recalc_edges.insert(ve_it);
				}
			}
		}
	}
	for (MyMesh::EdgeHandle e_h : recalc_edges)
	{
		computeError(e_h);
		float e = this->property(prop_e, e_h);
		if (e <= last_min)
		{
			HalfedgeHandle tmp_he_h = halfedge_handle(e_h, 0);
			VertexHandle tmp_to = to_vertex_handle(tmp_he_h);

			MyMesh::Point old_V = this->point(tmp_to);

			MyMesh::Point tmp_V = this->property(prop_v, e_h);

			set_point(tmp_to, tmp_V);
			bool collapse_ok = is_collapse_ok(tmp_he_h);
			set_point(tmp_to, old_V);

			if (collapse_ok) {
				found = true;
				last_min = e;
				last_handle = e_h;
			}
		}

	}

	return true;
}
void MyMesh::record() 
{
	std::vector<MyMesh::Point> points;
	std::vector<MyMesh::Point> normals;
	std::vector<unsigned int> indices;

	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		points.push_back(point(*v_it));
		normals.push_back(normal(*v_it));
	}
	for (MyMesh::FaceIter f_it = faces_begin(); f_it != faces_end(); ++f_it)
	{
		for (MyMesh::FaceVertexIter fv_it = fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
			indices.push_back(fv_it->idx());
		}
	}
	record_vertices.push_back(points);
	record_normals.push_back(normals);
	record_indices.push_back(indices);
}

#include <time.h>
void MyMesh::simplification()
{
	time_t total_s_time = 0;
	time_t s_time = clock();

	last_min = 0;
	use_last = false;

	if (record_vertices.empty())
	{
		int i = 0;
		record();
		for (int j = 0; j < 200;j++)
		{
			int n = std::min(1499, (int)(n_edges() * 0.016)) + 1;
			for (int k = 0; k < n; k++)
			{
				++i;
				if (collapse() == false)
				{
					garbage_collection();
					record();
					return;
				}
			}

			garbage_collection();
			record();

			time_t e_time = clock();
			total_s_time += e_time - s_time;
			std::cout << j << " : " << n << ", "<< e_time - s_time << "ms , avg : " << total_s_time / float(i)<< "ms                \r";
			s_time = e_time;
		}
	}
}

void MyMesh::generateLeastSquareMesh(std::vector<MyMesh::Point>& points, int control_num)
{
	//refer: https://github.com/shizsun0609tw/Least-Square-Meshes

	const int N(n_vertices());

	std::map<VertexHandle, int> vertices;
	std::set<int> control_indexs;
	std::vector<VertexHandle> control_points;

	if (control_num > N)
		control_num = N;
	while (control_indexs.size() < control_num)
	{
		control_indexs.insert(rand() % N);
	}

	int idx(0);
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it, ++idx)
	{
		vertices.insert(std::pair<VertexHandle, int>(v_it.handle(), idx));
		//vertices[v_it.handle()] = idx;
		if (control_indexs.find(idx) != control_indexs.end())
		{
			control_points.push_back(v_it.handle());
		}
	}

	const int M(control_points.size());

	// Solve A.T Ax = A.T b
	Eigen::SparseMatrix<double> A(N + M, N);
	Eigen::SparseMatrix<double> b(N + M, 3);
	std::vector<Eigen::Triplet<double>> triplet_list_A, triplet_list_b;

	// Laplacian
	for (auto it = vertices.begin(); it != vertices.end(); it++)
	{
		//i == j
		int i = it->second;	//col
		int j = it->second;	//row
		triplet_list_A.push_back(Eigen::Triplet<double>(j, i, 1.0));

		int d_j = 0;
		for (MyMesh::VertexVertexIter vv_it = vv_iter(it->first); vv_it.is_valid(); ++vv_it)
			d_j++;

		for (MyMesh::VertexVertexIter vv_it = vv_iter(it->first); vv_it.is_valid(); ++vv_it)
		{
			i = vertices[vv_it];
			triplet_list_A.push_back(Eigen::Triplet<double>(j, i, -1.0 / (double)d_j));
		}
	}

	// Constraint
	for (int j = 0; j < control_points.size(); j++)
	{
		int k = N + j;
		int i = vertices[control_points[j]];	//index
		MyMesh::Point c_point = point(control_points[j]);

		triplet_list_A.push_back(Eigen::Triplet<double>(k, i, 1.0));

		triplet_list_b.push_back(Eigen::Triplet<double>(k, 0, c_point[0]));
		triplet_list_b.push_back(Eigen::Triplet<double>(k, 1, c_point[1]));
		triplet_list_b.push_back(Eigen::Triplet<double>(k, 2, c_point[2]));
	}

	//fullfill A and b
	A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());
	b.setFromTriplets(triplet_list_b.begin(), triplet_list_b.end());

	Eigen::SparseMatrix<double> ATA = A.transpose() * A;
	Eigen::SparseMatrix<double> ATb = A.transpose() * b;

	//
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(ATA);
	Eigen::MatrixXd x = solver.solve(ATb);

	//
	points.reserve(N);
	for (auto it = vertices.begin(); it != vertices.end(); it++)
	{
		int idx = it->second;
		points[idx][0] = x.row(idx).col(0).value();
		points[idx][1] = x.row(idx).col(1).value();
		points[idx][2] = x.row(idx).col(2).value();
	}
}

inline double cot(Eigen::Vector3d v, Eigen::Vector3d w) {
	if (v == w)
		return std::numeric_limits<double>::infinity();
	return(v.dot(w) / v.cross(w).norm());
};
double MyMesh::computeWeight(MyMesh::HalfedgeHandle& heh)
{
	GLdouble alpha, beta, weight;
	MyMesh::Point pFrom = point(from_vertex_handle(heh));
	MyMesh::Point pTo = point(to_vertex_handle(heh));
	MyMesh::Point p1 = point(opposite_vh(heh));
	MyMesh::Point p2 = point(opposite_he_opposite_vh(heh));

	OpenMesh::Vec3d v1 = (OpenMesh::Vec3d)(p1 - pFrom); v1.normalize();
	OpenMesh::Vec3d v2 = (OpenMesh::Vec3d)(p1 - pTo); v2.normalize();
	alpha = std::acos(clamp(OpenMesh::dot(v1, v2), -1.0, 1.0));

	Eigen::Vector3d v_1(v1[0], v1[1], v1[2]);
	Eigen::Vector3d v_2(v2[0], v2[1], v2[2]);

	v1 = (OpenMesh::Vec3d)(p2 - pFrom); v1.normalize();
	v2 = (OpenMesh::Vec3d)(p2 - pTo); v2.normalize();
	beta = std::acos(clamp(OpenMesh::dot(v1, v2), -1.0, 1.0));

	Eigen::Vector3d v_3(v1[0], v1[1], v1[2]);
	Eigen::Vector3d v_4(v2[0], v2[1], v2[2]);

	//if (std::sin(alpha) == 0 || std::sin(beta) == 0)
		//	return FLT_MAX;
		//return std::cos(alpha) / std::sin(alpha) + std::cos(beta) / std::sin(beta);
	
	return cot(v_1, v_2) + cot(v_3, v_4);
}
void MyMesh::degenerateLeastSquareMesh(double W0_L, double W0_H, double S_L, int iterations)
{
	const int N(n_vertices());
	std::map<VertexHandle, int> vertices;
	std::vector<double> A0(N), At(N);
	{
		int idx(0);
		for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it, ++idx)
		{
			vertices.insert(std::make_pair(v_it.handle(), idx));
			A0[idx] = 0.0;
			for (MyMesh::VertexFaceIter vf_it = vf_iter(v_it); vf_it.is_valid(); ++vf_it)
			{
				A0[idx] += calc_face_area(vf_it);
			}
			At[idx] = A0[idx];
		}
	}

	std::vector<double> W_H(N, W0_H);

	double W_L(0.0);
	double avg_face_area = 0.0;
	for (auto it = faces_begin(); it != faces_end(); ++it)
	{
		avg_face_area += calc_face_area(it);
	}
	avg_face_area /= (double)n_faces();

	W_L = W0_L * sqrt(avg_face_area);

	//W_L = W0_L;
	for (int t(0); t < iterations; t++)
	{
		std::vector<MyMesh::Point> degeneration_points;
		degeneration_points.resize(N);

		// using all vertices to be control points
		Eigen::SparseMatrix<double> A(N + N, N);
		Eigen::SparseMatrix<double> b(N + N, 3);

		//Eigen::VectorXd bx(N + N, 1); bx.setZero();
		//Eigen::VectorXd by(N + N, 1); by.setZero();
		//Eigen::VectorXd bz(N + N, 1); bz.setZero();
		std::vector<Eigen::Triplet<double>> triplet_list_A, triplet_list_b, triplet_list_bx, triplet_list_by, triplet_list_bz;
	
		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			// Laplacian
			int i = it->second;	//row
			int j = it->second;	//col

			double w_ij = 0.0;
			for (MyMesh::VertexOHalfedgeIter voh_it = voh_iter(it->first); voh_it.is_valid(); ++voh_it)
			{
				double w_ik = computeWeight(voh_it.handle());

				int k = vertices[to_vertex_handle(voh_it)];
				triplet_list_A.push_back(Eigen::Triplet<double>(i, k, W_L * w_ik));

				w_ij += -w_ik;
			}
			triplet_list_A.push_back(Eigen::Triplet<double>(i, j, W_L * w_ij));

			// Constraint
			triplet_list_A.push_back(Eigen::Triplet<double>(N + i, j, W_H[i]));

			MyMesh::Point vi = point(it->first);
			triplet_list_b.push_back(Eigen::Triplet<double>(N + i, 0, W_H[i] * vi[0]));
			triplet_list_b.push_back(Eigen::Triplet<double>(N + i, 1, W_H[i] * vi[1]));
			triplet_list_b.push_back(Eigen::Triplet<double>(N + i, 2, W_H[i] * vi[2]));

			//triplet_list_bx.push_back(Eigen::Triplet<double>(N + i, 0, W_H[i] * vi[0]));
			//triplet_list_by.push_back(Eigen::Triplet<double>(N + i, 0, W_H[i] * vi[1]));
			//triplet_list_bz.push_back(Eigen::Triplet<double>(N + i, 0, W_H[i] * vi[2]));

			//bx(N + i, 0) = W_H[i] * vi[0];
			//by(N + i, 0) = W_H[i] * vi[1];
			//bz(N + i, 0) = W_H[i] * vi[2];
		}
		//fullfill A and b
		A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());
		b.setFromTriplets(triplet_list_b.begin(), triplet_list_b.end());

	/*	bx.setFromTriplets(triplet_list_bx.begin(), triplet_list_bx.end());
		by.setFromTriplets(triplet_list_by.begin(), triplet_list_by.end());
		bz.setFromTriplets(triplet_list_bz.begin(), triplet_list_bz.end());*/

		//std::cout << A << std::endl;
		//std::cout << bx << std::endl;

		Eigen::SparseMatrix<double> ATA = A.transpose() * A;
		Eigen::SparseMatrix<double> ATb = A.transpose() * b;

		////
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(ATA);
		//Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver(ATA);
		Eigen::MatrixXd x = solver.solve(ATb);
		//Eigen::VectorXd newx = solver.solve(ATbx);
		//Eigen::VectorXd newy = solver.solve(ATby);
		//Eigen::VectorXd newz = solver.solve(ATbz);

		std::cout << x(0, 0) << std::endl;

		//update
		W_L *= S_L;
		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			int i = it->second;
			float xx = x(i, 0);
			float yy = x(i, 1);
			float zz = x(i, 2);

			//float xx = newx(i, 0);
			//float yy = newy(i, 0);
			//float zz = newz(i, 0);
			set_point(it->first, MyMesh::Point(xx, yy, zz));

			degeneration_points[i] = MyMesh::Point(xx, yy, zz);
		}

		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			int i = it->second;
			At[i] = 0.0;
			for (MyMesh::VertexFaceIter vf_it = vf_iter(it->first); vf_it.is_valid(); ++vf_it)
			{
				At[i] += calc_face_area(vf_it);
			}

			double tmpW_H = W0_H * pow(A0[i] / At[i] + 0.6, 2.0);
			//double tmpW_H = W0_H * sqrt(A0[i] / At[i]);
			//W_H[i] = W0_H * sqrt(A0[i] / At[i]);
			if (tmpW_H > W_H[i]) {
				W_H[i] = tmpW_H;
			}
		}
		this->degeneration_vertices.push_back(degeneration_points);
	}
}

void MyMesh::degenerationMeshToLine()
{
	add_property(prop_sk_ve, "prop_sk_ve");// vertex error
	add_property(prop_sk_vl, "prop_sk_vl");// vertex adjacent len
	add_property(prop_sk_vQ, "prop_sk_vQ");//

	std::vector<MyMesh::VertexHandle> sk_vertices;
	std::map<MyMesh::VertexHandle, std::vector<MyMesh::SKHalfedge>> outHalfedges;
	std::map< MyMesh::VertexHandle, std::vector<SKFace>> outFaces;
	inv_avg_length = 0;
	for (MyMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) {
		float length = this->calc_edge_length(e_it);
		inv_avg_length += length;
	}
	inv_avg_length = (float)n_edges() / inv_avg_length;

	std::cout << inv_avg_length << std::endl;

	for (MyMesh::VertexIter v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it) {
		sk_vertices.push_back(v_it);
		initSKVertexErrorQuadric(outHalfedges, v_it); // only need to call this once for initialize
		computeSKVertexError(outHalfedges, v_it); // update this
	}
	for (MyMesh::VertexIter v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it) {
		std::vector<SKHalfedge>& halfedges = outHalfedges[v_it];
		for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
			computeSKEdgeCost(he_it); // update this
		}

		std::pair<std::map<VertexHandle, std::vector<SKFace>>::iterator, bool> ret;
		ret = outFaces.insert(std::pair<VertexHandle, std::vector<SKFace>>(v_it, std::vector<SKFace>()));

		std::vector<SKFace>& faces = ret.first->second;
		SKFace tmpFace;
		tmpFace.from = v_it;
		for (auto vf_it = vf_iter(v_it); vf_it.is_valid(); ++vf_it) {
			int idx = 0;
			for (auto vfv_it = fv_iter(vf_it); vfv_it.is_valid(); ++vfv_it) {
				if (vfv_it.handle() != v_it) {
					tmpFace.to[idx] = vfv_it;
					++idx;
				}
			}
			faces.push_back(tmpFace);
		}
	}

	time_t total_s_time = 0;
	time_t s_time = clock();

	last_min = 0;
	use_last = false;

	degeneration_indices.clear();
	
	std::vector<unsigned int> indices;
	for (auto hes_it = outHalfedges.begin(); hes_it != outHalfedges.end(); ++hes_it) {
		std::vector<MyMesh::SKHalfedge>& halfedges = hes_it->second;
		for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
			indices.push_back(he_it->to.idx());
			indices.push_back(he_it->from.idx());
		}
	}
	degeneration_indices.push_back(indices);

	int n = 500;
	for (int j = 1; j <= n; j++)
	{
		for (int k = 0; k < 100; k++) {
			if (collapseToLine(sk_vertices, outHalfedges, outFaces) == false)
			{
				j = n + 1;
				break;
			}
		}
		time_t e_time = clock();
		total_s_time += e_time - s_time;
		std::cout << j << ", " << e_time - s_time << "ms , avg : " << total_s_time / float(j) << "ms                \r";
		s_time = e_time;

		// store
		indices.clear();
		for (auto hes_it = outHalfedges.begin(); hes_it != outHalfedges.end(); ++hes_it) {
			std::vector<MyMesh::SKHalfedge>& halfedges = hes_it->second;
			for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
				indices.push_back(he_it->to.idx());
				indices.push_back(he_it->from.idx());
			}
		}
		degeneration_indices.push_back(indices);
	}

	int count = 0;
	for (auto& ofaces : outFaces) {
		count += ofaces.second.size();
	}
	std::cout << "\n faces: " << count << std::endl;
}

bool MyMesh::collapseToLine(std::vector<VertexHandle>& sk_vertices, 
	std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, std::map<VertexHandle, std::vector<SKFace>>& of_map)
{
	std::vector<SKHalfedge>::iterator sk_he_it;
	std::vector<VertexHandle>::iterator sk_v_it;
	float min = FLT_MAX;

	bool found = false;

	for (auto v_it = sk_vertices.begin(); v_it != sk_vertices.end() && !found; ++v_it) {
		std::vector<SKHalfedge>& halfedges = ohe_map[*v_it];

		for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {

			float cost = he_it->cost;
			if (cost < min)
			{
				if (edge_is_collapse_ok(of_map, ohe_map, he_it)) {
					found = true;
					min = cost;

					sk_v_it = v_it;
					sk_he_it = he_it;

					if (min <= this->last_min) {
						break;
					}
				}
			}
		}
	}

	if (!found) {
		std::cout << "Not Found Break" << std::endl;
		return false;
	}

	last_min = min;

	VertexHandle from = sk_he_it->from;
	VertexHandle to = sk_he_it->to;

	edge_collapse(of_map, ohe_map, sk_he_it);
	sk_vertices.erase(sk_v_it);

	//recompute quadrics & edge errors
	this->property(prop_sk_vQ, to) = this->property(prop_sk_vQ, to) + this->property(prop_sk_vQ, from);

	computeSKVertexError(ohe_map, to);
	std::vector<SKHalfedge>& halfedges = ohe_map[to];
	for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
		computeSKVertexError(ohe_map, he_it->to);

		std::vector<SKHalfedge>& to_halfedges = ohe_map[he_it->to];

		for (auto to_he_it = to_halfedges.begin(); to_he_it != to_halfedges.end(); ++to_he_it)
		{
			computeSKEdgeCost(to_he_it);
			float cost = to_he_it->cost;

			if (cost < last_min)
			{
				last_min = cost;
			}

			std::vector<SKHalfedge>& to_to_halfedges = ohe_map[to_he_it->to];
			for (auto to_to_he_it = to_to_halfedges.begin(); to_to_he_it != to_to_halfedges.end(); ++to_to_he_it)
			{
				computeSKEdgeCost(to_to_he_it);
				float cost = to_to_he_it->cost;

				if (cost < last_min)
				{
					last_min = cost;
				}
			}
		}
	}

	return true;
}

void MyMesh::initSKVertexErrorQuadric(std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, MyMesh::VertexHandle v_h)
{
	Point p = point(v_h);
	Eigen::Vector3d v(p[0], p[1], p[2]);

	Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
	Eigen::MatrixXd K(3, 4);

	std::pair<std::map<VertexHandle, std::vector<SKHalfedge>>::iterator, bool> ret;
	ret = ohe_map.insert(std::pair<VertexHandle, std::vector<SKHalfedge>>(v_h, std::vector<SKHalfedge>()));

	std::vector<SKHalfedge>& halfedges = ret.first->second;

	for (VertexOHalfedgeIter voh_it = voh_iter(v_h); voh_it.is_valid(); ++voh_it) {

		VertexHandle v_h1 = to_vertex_handle(voh_it);

		Point p1 = point(v_h1);
		Eigen::Vector3d a(p1[0] - p[0], p1[1] - p[1], p1[2] - p[2]);
		a.normalize();
		Eigen::Vector3d b = a.cross(v);

		K << 0, -a[2], a[1], -b[0], \
			a[2], 0, -a[0], -b[1], \
			- a[1], a[0], 0, -b[2];

		Q += K.transpose() * K;

		halfedges.push_back(SKHalfedge(v_h, v_h1, 0));
	}

	this->property(prop_sk_vQ, v_h) = Q;
}

void MyMesh::computeSKVertexError(std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, MyMesh::VertexHandle v_h)
{
	Point p = point(v_h);
	Eigen::Vector4d v(p[0], p[1], p[2], 1);

	Eigen::Matrix4d& Q = this->property(prop_sk_vQ, v_h);

	float error = abs(v.dot(Q * v));
	float adj = 0;

	std::vector<SKHalfedge>& halfedges = ohe_map[v_h];
	for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
		if (!status(he_it->to).deleted()) {
			adj += (p - point(he_it->to)).length();
		}
	}

	this->property(prop_sk_ve, v_h) = error;
	this->property(prop_sk_vl, v_h) = adj;
}

void MyMesh::computeSKEdgeCost(std::vector<SKHalfedge>::iterator sk_he_it)
{
	const float w_a = 1.0;
	const float w_b = 10;

	VertexHandle v_h0 = sk_he_it->from; // from
	VertexHandle v_h1 = sk_he_it->to; // to

	float Fa = this->property(prop_sk_ve, v_h0) + this->property(prop_sk_ve, v_h1);

	float adj_distance = property(prop_sk_vl, v_h0);

	float length = (point(v_h0) - point(v_h1)).length();
	float Fb = inv_avg_length * length * adj_distance;

	sk_he_it->cost = w_a * Fa + w_b * Fb;
}

bool MyMesh::edge_is_collapse_ok(std::map<VertexHandle, std::vector<SKFace>>& of_map,
	std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, std::vector<SKHalfedge>::iterator sk_he_it)
{
	return (of_map[sk_he_it->from].size() > 0 && of_map[sk_he_it->to].size() > 0);
}

bool MyMesh::edge_collapse(std::map<VertexHandle, std::vector<SKFace>>& of_map,
	std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, std::vector<SKHalfedge>::iterator sk_he_it)
{
	VertexHandle v_h0 = sk_he_it->from;
	VertexHandle v_h1 = sk_he_it->to;

	auto v0_he_iter = ohe_map.find(v_h0);
	auto v1_he_iter = ohe_map.find(v_h1);

	std::vector<SKHalfedge>& halfedges0 = v0_he_iter->second;
	std::vector<SKHalfedge>& halfedges1 = v1_he_iter->second;

	for (auto he_it0 = halfedges0.begin(); he_it0 != halfedges0.end(); ++he_it0) {

		std::vector<SKHalfedge>& halfedges0_i = ohe_map[he_it0->to];

		// remove halfedge
		for (auto t_he_it = halfedges0_i.begin(); t_he_it != halfedges0_i.end(); ++t_he_it) {
			if (t_he_it->to == v_h0) {
				halfedges0_i.erase(t_he_it);
				break;
			}
		}

		// append edge
		if (he_it0->to != v_h1) {
			// check if this edge exist
			bool found = false;
			for (auto he_it1 = halfedges1.begin(); he_it1 != halfedges1.end(); ++he_it1) {
				if (he_it1->to == he_it0->to) {
					found = true;
					break;
				}
			}
			if (!found) {
				halfedges1.push_back(SKHalfedge(v_h1, he_it0->to));
				halfedges0_i.push_back(SKHalfedge(he_it0->to, v_h1));
			}
			
		}
	}
	ohe_map.erase(v0_he_iter);

	// remove & add faces
	auto v0_of_iter = of_map.find(v_h0);
	auto v1_of_iter = of_map.find(v_h1);

	std::vector<SKFace>& outfaces0 = v0_of_iter->second;
	std::vector<SKFace>& outfaces1 = v1_of_iter->second;

	// remove face
	for (auto of_it0 = outfaces0.begin(); of_it0 != outfaces0.end(); ++of_it0) {
		for (int idx = 0; idx < 2; idx++) {
			std::vector<SKFace>& outfaces0_i = of_map[of_it0->to[idx]];
			for (auto t_of_it = outfaces0_i.begin(); t_of_it != outfaces0_i.end();) {
				if (t_of_it->to[0] == v_h0 || t_of_it->to[1] == v_h0) {
					t_of_it = outfaces0_i.erase(t_of_it);
				}
				else {
					++t_of_it;
				}
			}
		}

		// add face
		SKFace tmpFace;
		if ((of_it0->to[0] != v_h1) && (of_it0->to[1] != v_h1)) {
			std::vector<SKFace>& outfaces0_0 = of_map[of_it0->to[0]];
			std::vector<SKFace>& outfaces0_1 = of_map[of_it0->to[1]];

			tmpFace.from = v_h1;
			tmpFace.to[0] = of_it0->to[0];
			tmpFace.to[1] = of_it0->to[1];
			outfaces1.push_back(tmpFace);

			tmpFace.from = of_it0->to[0];
			tmpFace.to[0] = of_it0->to[1];
			tmpFace.to[1] = v_h1; 
			outfaces0_0.push_back(tmpFace);

			tmpFace.from = of_it0->to[1];
			tmpFace.to[0] = v_h1;
			tmpFace.to[1] = of_it0->to[0];
			outfaces0_1.push_back(tmpFace);
		}
	}
	of_map.erase(v0_of_iter);

	return false;
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
	if (LoadModel(fileName))
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

		std::vector<MyMesh::Point> vertices;
		std::vector<MyMesh::Normal> normals;
		//vertices.reserve(mesh.n_vertices());
		//normals.reserve(mesh.n_vertices());
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			vertices.push_back(mesh.point(*v_it));
			normals.push_back(mesh.normal(*v_it));
		}

		std::vector<unsigned int> indices;
		//indices.reserve(mesh.n_faces() * 3);
		for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		{
			for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{
				indices.push_back(fv_it->idx());
			}
		}
		//std::cout << "vertex number: " << mesh.n_vertices() << std::endl;
		//std::cout << "edge number: " << mesh.n_edges() << std::endl;
		//std::cout << "half edge number: " << mesh.n_halfedges() << std::endl;
		//std::cout << "face number: " << mesh.n_faces() << std::endl;

		LoadToShader(vertices, normals, indices);
		return true;
	}
	return false;
}

void GLMesh::Render()
{
	glBindVertexArray(this->vao.vao);

	if(this->render_mode == 0)
		glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
	else
		glDrawElements(GL_LINES, this->vao.element_amount, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
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

			mesh.computeErrorQuadrics();
			mesh.computeError();
		}
		return true;
	}

	return false;
}

void GLMesh::LoadToShader(
	std::vector<MyMesh::Point>& vertices, std::vector<MyMesh::Normal>& normals, std::vector<unsigned int>& indices, int mode)
{
	this->render_mode = mode;

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

void GLMesh::simplification(float ratio)
{
	if (mesh.record_vertices.empty())
		mesh.simplification();

	int idx = (int)((1.0f - ratio) * (mesh.record_vertices.size()-1));
	//reload
	LoadToShader(
		mesh.record_vertices[idx],
		mesh.record_normals[idx],
		mesh.record_indices[idx]);
}
bool GLMesh::exportSimplificationMesh(float ratio)
{
	if (mesh.record_vertices.empty())
	{
		puts("Not yet simplified");
		return false;
	}


	int idx = (int)((1.0f - ratio) * (mesh.record_vertices.size() - 1));

	MyMesh new_mesh;

	std::vector<MyMesh::VertexHandle> v_handles;
	for (OpenMesh::Vec3f v : mesh.record_vertices[idx])
	{
		v_handles.push_back(new_mesh.add_vertex(v));
	}
	std::vector<MyMesh::VertexHandle>  face_vhandles;
	for (int i = 0 ; i < mesh.record_indices[idx].size(); i+=3)
	{
		face_vhandles.clear();
		face_vhandles.push_back(v_handles[mesh.record_indices[idx][i]]);
		face_vhandles.push_back(v_handles[mesh.record_indices[idx][i+1]]);
		face_vhandles.push_back(v_handles[mesh.record_indices[idx][i+2]]);
		new_mesh.add_face(face_vhandles);
	}

	// write mesh to output.obj
	try
	{
		if (!OpenMesh::IO::write_mesh(new_mesh, "output.obj"))
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

void GLMesh::generateLeastSquareMesh(int control_num)
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

	mesh.generateLeastSquareMesh(vertices, control_num);

	LoadToShader(vertices, normals, indices);
}

void GLMesh::degenerateLeastSquareMesh(float ratio)
{
	std::vector<MyMesh::Normal> normals;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		normals.push_back(mesh.normal(*v_it));
	}
	std::vector<unsigned int> indices;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			indices.push_back(fv_it->idx());

	if (mesh.degeneration_vertices.empty())
		mesh.degenerateLeastSquareMesh(0.001, 1.0, 4.0, 20);

	int idx = (int)((1.0f - ratio) * (mesh.degeneration_vertices.size() - 1));

	std::cout << "idx : " << idx << "      \r";
	//reload
	LoadToShader(
		mesh.degeneration_vertices[idx],
		normals,
		indices);
}

void GLMesh::degenerationMeshToLineSlider(float ratio)
{
	if (deg_simp_mesh.degeneration_indices.empty())
	{
		puts("Not yet simplified");
		return;
	}

	int idx = (int)((1.0f - ratio) * (deg_simp_mesh.degeneration_indices.size() - 1));

	std::vector<MyMesh::Point> vertices;
	std::vector<MyMesh::Normal> normals;
	for (MyMesh::VertexIter v_it = deg_simp_mesh.vertices_begin(); v_it != deg_simp_mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(deg_simp_mesh.point(*v_it));
		normals.push_back(mesh.normal(*v_it));
	}

	LoadToShader(vertices, normals, deg_simp_mesh.degeneration_indices[idx], 1);

}

void GLMesh::degenerationMeshToLine(float ratio)
{
	if (mesh.degeneration_vertices.empty())
	{
		puts("Not yet simplified");
		return;
	}

	int idx = (int)((1.0f - ratio) * (mesh.degeneration_vertices.size() - 1));
	std::cout << "\nstart with idx = " << idx << std::endl;

	deg_simp_mesh.clear();

	std::vector<MyMesh::VertexHandle> v_handles;
	for (OpenMesh::Vec3f v : mesh.degeneration_vertices[idx])
	{
		v_handles.push_back(deg_simp_mesh.add_vertex(v));
	}
	std::vector<MyMesh::VertexHandle>  face_vhandles;

	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
		face_vhandles.clear();
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
			face_vhandles.push_back(v_handles[fv_it->idx()]);
		}
		deg_simp_mesh.add_face(face_vhandles);
	}

	deg_simp_mesh.degenerationMeshToLine();

	this->degenerationMeshToLineSlider(0);
}

#pragma endregion

MyMesh::SKHalfedge::SKHalfedge(VertexHandle f, VertexHandle t, float c)
{
	from = f;
	to = t;
	cost = c;
}
