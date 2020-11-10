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
	//request_halfedge_status();
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

		if (e0 > e1&& e0 > e2) {
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
void MyMesh::recordSimplification(
	std::vector<std::vector<MyMesh::Point>>& record_vertices,
	std::vector<std::vector<MyMesh::Normal>>& record_normals,
	std::vector<std::vector<unsigned int>>& record_indices)
{
	std::vector<MyMesh::Point> points;
	std::vector<MyMesh::Normal> normals;
	std::vector<unsigned int> indices;

	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		points.push_back(point(*v_it));
		//normals.push_back(normal(*v_it));
		normals.push_back(Normal(0,1,0));
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
void MyMesh::simplification(
	std::vector<std::vector<MyMesh::Point>>& record_vertices,
	std::vector<std::vector<MyMesh::Normal>>& record_normals,
	std::vector<std::vector<unsigned int>>& record_indices)
{
	time_t total_s_time = 0;
	time_t s_time = clock();

	last_min = 0;
	use_last = false;
	last_handle = MyMesh::EdgeHandle();

	if (record_vertices.empty())
	{
		int i = 0;
		recordSimplification(record_vertices, record_normals, record_indices);
		for (int j = 0; j < 200; j++)
		{
			int n = std::min(1499, (int)(n_edges() * 0.016)) + 1;
			for (int k = 0; k < n; k++)
			{
				++i;
				if (collapse() == false)
				{
					garbage_collection();
					recordSimplification(record_vertices, record_normals, record_indices);
					return;
				}
			}

			garbage_collection();
			recordSimplification(record_vertices, record_normals, record_indices);

			time_t e_time = clock();
			total_s_time += e_time - s_time;
			std::cout << j << " : " << n << ", " << e_time - s_time << "ms , avg : " << total_s_time / float(i) << "ms                \r";
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

double MyMesh::computeLaplacianWeight(MyMesh::HalfedgeHandle& heh)
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
void MyMesh::degenerateLeastSquareMesh(std::vector<std::vector<MyMesh::Point>>& record_vertices, double W0_L, double W0_H, double S_L, int iterations)
{
	//record initial
	{
		std::vector<MyMesh::Point> initial_points;
		for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
		{
			initial_points.push_back(point(v_it));
		}
		record_vertices.push_back(initial_points);
	}


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

		std::vector<Eigen::Triplet<double>> triplet_list_A, triplet_list_b, triplet_list_bx, triplet_list_by, triplet_list_bz;

		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			// Laplacian
			int i = it->second;	//row
			int j = it->second;	//col

			double w_ij = 0.0;
			for (MyMesh::VertexOHalfedgeIter voh_it = voh_iter(it->first); voh_it.is_valid(); ++voh_it)
			{
				double w_ik = computeLaplacianWeight(voh_it.handle());

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
		}
		//fullfill A and b
		A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());
		b.setFromTriplets(triplet_list_b.begin(), triplet_list_b.end());

		Eigen::SparseMatrix<double> ATA = A.transpose() * A;
		Eigen::SparseMatrix<double> ATb = A.transpose() * b;

		////
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(ATA);
		Eigen::MatrixXd x = solver.solve(ATb);

		std::cout << x(0, 0) << std::endl;
		if (isnan(x(0, 0)))
			return;

		//update
		W_L *= S_L;
		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			int i = it->second;
			MyMesh::Point new_point(x(i, 0), x(i, 1), x(i, 2));

			set_point(it->first, new_point);
			degeneration_points[i] = new_point;
		}

		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			int i = it->second;
			At[i] = 0.0;
			for (MyMesh::VertexFaceIter vf_it = vf_iter(it->first); vf_it.is_valid(); ++vf_it)
			{
				At[i] += calc_face_area(vf_it);
			}

			//double tmpW_H = W0_H * pow(A0[i] / At[i], 3.0);
			double tmpW_H = W0_H * sqrt(A0[i] / At[i]);
			//W_H[i] = W0_H * sqrt(A0[i] / At[i]);
			if (tmpW_H > W_H[i]) {
				W_H[i] = tmpW_H;
			}
		}
		record_vertices.push_back(degeneration_points);
	}
}
void MyMesh::degenerationMeshToLine(
	std::vector<std::vector<unsigned int>>& degeneration_indices, std::vector<MyMesh::Point>& origin_vertices)
{
	add_property(prop_sk_vl, "prop_sk_vl");// vertex adjacent len
	add_property(prop_sk_vQ, "prop_sk_vQ");//

	std::vector<MyMesh::VertexHandle> sk_vertices;
	std::map< MyMesh::VertexHandle, std::vector<MyMesh::VertexHandle>> boundaries;
	std::map<MyMesh::VertexHandle, std::vector<MyMesh::SKHalfedge>> outHalfedges;
	std::map< MyMesh::VertexHandle, std::vector<SKFace>> outFaces;

	sk_face_count = 0;

	for (MyMesh::VertexIter v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it) {
		sk_vertices.push_back(v_it);
		boundaries.insert(std::pair < MyMesh::VertexHandle, std::vector<MyMesh::VertexHandle>>(v_it, std::vector<MyMesh::VertexHandle>()));
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
			sk_face_count += 1;
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

	int n = 100;
	for (int j = 1; j <= n; j++)
	{
		for (int k = 0; k < 1000; k++) {
			if (collapseToLine(sk_vertices, outHalfedges, outFaces, boundaries) == false)
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
	std::cout << "\n faces: " << sk_face_count << " " << count << std::endl;
	// embedding refinement
	std::map<VertexHandle, Point> displacements;
	for (auto& bound : boundaries) {
		std::vector<VertexHandle>& v_loop = bound.second;
		v_loop.push_back(bound.first);

		Point disp(0, 0, 0);
		double total = 0;
		for (auto v_it = v_loop.begin(); v_it != v_loop.end(); ++v_it) {

			VertexHandle v_h = *v_it;
			Point p_c = point(v_h);
			Point p_o = origin_vertices[v_it->idx()];

			std::set<EdgeHandle> two_ring;
			double length = 0;

			for (VertexOHalfedgeIter voe_it = voh_iter(v_h); voe_it.is_valid(); ++voe_it) {
				VertexHandle vov_h = to_vertex_handle(voe_it);
				for (VertexEdgeIter vov_e_it = ve_iter(vov_h); vov_e_it.is_valid(); ++vov_e_it) {
					two_ring.insert(vov_e_it.handle());
				}
			}

			for (auto e_it = two_ring.begin(); e_it != two_ring.end(); ++e_it) {
				length += calc_edge_length(*e_it);
			}

			total += length;
			disp += length * (p_c - p_o);
		}
		displacements[bound.first] = (total == 0) ? (Point(0, 0, 0)) : (disp / total);
	}
	for (auto v_it = sk_vertices.begin(); v_it != sk_vertices.end(); ++v_it) {
		VertexHandle v_h = *v_it;
		Point u = point(v_h);

		Point d = displacements[v_h];

		std::vector<SKHalfedge>& halfedges = outHalfedges[v_h];
		for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
			d += displacements[he_it->to];
		}

		d /= (halfedges.size() + 1);

		set_point(v_h, u - d);
	}

}
bool MyMesh::collapseToLine(std::vector<VertexHandle>& sk_vertices,
	std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, std::map<VertexHandle, std::vector<SKFace>>& of_map,
	std::map < VertexHandle, std::vector<VertexHandle>>& boundary_map)
{
	std::vector<SKHalfedge>::iterator sk_he_it;
	std::vector<VertexHandle>::iterator sk_v_it;

	double min = std::numeric_limits<double>::infinity();
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

	auto bound_it0 = boundary_map.find(from);
	auto bound_it1 = boundary_map.find(to);

	bound_it1->second.push_back(from);
	bound_it1->second.insert(bound_it1->second.end(), bound_it0->second.begin(), bound_it0->second.end());

	boundary_map.erase(bound_it0);

	//recompute quadrics & edge errors
	this->property(prop_sk_vQ, to) = (this->property(prop_sk_vQ, to) + this->property(prop_sk_vQ, from));

	computeSKVertexError(ohe_map, to);
	std::vector<SKHalfedge>& halfedges = ohe_map[to];
	for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
		computeSKVertexError(ohe_map, he_it->to);
	}
	for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
		computeSKEdgeCost(he_it);
		std::vector<SKHalfedge>& to_halfedges = ohe_map[he_it->to];
		for (auto to_he_it = to_halfedges.begin(); to_he_it != to_halfedges.end(); ++to_he_it)
		{
			computeSKEdgeCost(to_he_it);
			float cost = to_he_it->cost;

			if (cost < last_min)
			{
				last_min = cost;
			}
		}
	}
	return sk_face_count > 0;
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
		Eigen::Vector3d a((double)p1[0] - p[0], (double)p1[1] - p[1], (double)p1[2] - p[2]);
		a.normalize();
		Eigen::Vector3d b = a.cross(v);

		K << 0, -a[2], a[1], -b[0], \
			a[2], 0, -a[0], -b[1], \
			- a[1], a[0], 0, -b[2];

		Q += (K.transpose() * K);

		halfedges.push_back(SKHalfedge(v_h, v_h1, 0));
	}

	this->property(prop_sk_vQ, v_h) = Q;
}
void MyMesh::computeSKVertexError(std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, MyMesh::VertexHandle v_h)
{
	Point p = point(v_h);
	double adj = 0;

	std::vector<SKHalfedge>& halfedges = ohe_map[v_h];
	for (auto he_it = halfedges.begin(); he_it != halfedges.end(); ++he_it) {
		Point p01 = (p - point(he_it->to));
		adj += sqrt((double)p01[0] * (double)p01[0] + (double)p01[1] * (double)p01[1] + (double)p01[2] * (double)p01[2]);
	}

	this->property(prop_sk_vl, v_h) = adj;
}
void MyMesh::computeSKEdgeCost(std::vector<SKHalfedge>::iterator sk_he_it)
{
	const double w_a = 1000.0;
	const double w_b = 500.0;

	VertexHandle v_h0 = sk_he_it->from; // from
	VertexHandle v_h1 = sk_he_it->to; // to

	Eigen::Matrix4d& Q0 = this->property(prop_sk_vQ, v_h0);
	Eigen::Matrix4d& Q1 = this->property(prop_sk_vQ, v_h1);

	Point p0 = point(v_h0);
	Point p1 = point(v_h1);

	Eigen::Vector4d v1(p1[0], p1[1], p1[2], 1);

	double Fa = abs(v1.dot(Q0 * v1)) + abs(v1.dot(Q1 * v1));

	double adj_distance = property(prop_sk_vl, v_h0);

	Point p01 = (p1 - p0);
	double length = sqrt((double)p01[0] * (double)p01[0] + (double)p01[1] * (double)p01[1] + (double)p01[2] * (double)p01[2]);
	double Fb = length + adj_distance;

	sk_he_it->cost = w_a * Fa + w_b * Fb;
}
bool MyMesh::edge_is_collapse_ok(std::map<VertexHandle, std::vector<SKFace>>& of_map,
	std::map<VertexHandle, std::vector<SKHalfedge>>& ohe_map, std::vector<SKHalfedge>::iterator sk_he_it)
{
	std::vector<SKHalfedge>& halfedges0 = ohe_map[sk_he_it->from];
	std::vector<SKHalfedge>& halfedges1 = ohe_map[sk_he_it->to];

	std::vector<SKFace>& outFaces0 = of_map[sk_he_it->from];
	std::vector<SKFace>& outFaces1 = of_map[sk_he_it->to];
	return (outFaces0.size() > 0 && outFaces1.size() > 0);
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
					sk_face_count -= 1;
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
			sk_face_count += 1;

			tmpFace.from = of_it0->to[0];
			tmpFace.to[0] = of_it0->to[1];
			tmpFace.to[1] = v_h1;
			outfaces0_0.push_back(tmpFace);
			sk_face_count += 1;

			tmpFace.from = of_it0->to[1];
			tmpFace.to[0] = v_h1;
			tmpFace.to[1] = of_it0->to[0];
			outfaces0_1.push_back(tmpFace);
			sk_face_count += 1;
		}
	}

	sk_face_count -= outfaces0.size();
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

		//std::cout << "vertex number: " << mesh.n_vertices() << std::endl;
		//std::cout << "edge number: " << mesh.n_edges() << std::endl;
		//std::cout << "half edge number: " << mesh.n_halfedges() << std::endl;
		//std::cout << "face number: " << mesh.n_faces() << std::endl;

		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			initial_vertices.push_back(mesh.point(*v_it));
			initial_normals.push_back(mesh.normal(*v_it));
		}
		for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
			for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
				initial_indices.push_back(fv_it->idx());

		LoadToShader(initial_vertices, initial_normals, initial_indices);

		{
			glGenVertexArrays(1, &this->skeleton.vao);
			glBindVertexArray(this->skeleton.vao);

			glGenBuffers(3, this->skeleton.vbo);

			glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[0]);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glEnableVertexAttribArray(0);

			glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[1]);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glEnableVertexAttribArray(1);

			glGenBuffers(1, &this->skeleton.ebo);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->skeleton.ebo);

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
		}
	
		return true;
	}
	return false;
}

void GLMesh::renderMesh()
{
	glBindVertexArray(this->vao.vao);
	glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
void GLMesh::renderSkeleton()
{
	glBindVertexArray(this->skeleton.vao);
	glDrawElements(GL_LINES, this->skeleton.element_amount, GL_UNSIGNED_INT, 0);
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
void GLMesh::simplification()
{
	for (int i(0); i < simplification_vertices.size(); i++)
	{
		simplification_vertices[i].clear();
		simplification_normals[i].clear();
		simplification_indices[i].clear();
	}
	simplification_vertices.clear();
	simplification_normals.clear();
	simplification_indices.clear();

	mesh.simplification(
		simplification_vertices,
		simplification_normals,
		simplification_indices);

	simplification(0.0f);
}

void GLMesh::simplification(float ratio)
{
	if (simplification_vertices.empty())
		return;

	int idx = (int)((1.0f - ratio) * (simplification_vertices.size() - 1));
	//reload
	mesh.reloadMesh(simplification_vertices[idx], simplification_indices[idx]);
	LoadToShader();
}
void GLMesh::resetMesh()
{
	mesh.reloadMesh(initial_vertices, initial_indices);
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

void GLMesh::generateLeastSquareMesh(int control_num)
{
	std::vector<MyMesh::Point> vertices;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));
	}
	std::vector<unsigned int> indices;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			indices.push_back(fv_it->idx());

	mesh.generateLeastSquareMesh(vertices, control_num);
	mesh.reloadMesh(vertices, indices);

	LoadToShader();
}

void GLMesh::degenerateLeastSquareMesh()
{
	for (auto& d : degeneration_vertices)
		d.clear();
	degeneration_vertices.clear();
	degeneration_indices.clear();

	mesh.degenerateLeastSquareMesh(degeneration_vertices, 0.005, 1.2, 2.8, 20);

	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			degeneration_indices.push_back(fv_it->idx());

	degenerateLeastSquareMesh(0.0f);
}

void GLMesh::degenerateLeastSquareMesh(float ratio)
{
	if (degeneration_vertices.empty())
		return;

	int idx = (int)((1.0f - ratio) * (degeneration_vertices.size() - 1));

	//reload
	mesh.reloadMesh(degeneration_vertices[idx], degeneration_indices);
	LoadToShader();
}

void GLMesh::degenerationMeshToLine(float ratio)
{
	if (skeleton_indices.empty())
	{
		return;
	}

	int idx = (int)((1.0f - ratio) * (skeleton_indices.size() - 1));

	this->skeleton.element_amount = skeleton_indices[idx].size();

	glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * skeleton_vertices.size(), &skeleton_vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->skeleton.vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * skeleton_normal.size(), &skeleton_normal[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->skeleton.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * skeleton_indices[idx].size(), &skeleton_indices[idx][0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

void GLMesh::degenerationMeshToLine()
{
	if (degeneration_vertices.empty())
		return;

	for (auto& s : this->skeleton_indices)
		s.clear();
	this->skeleton_indices.clear();
	this->skeleton_vertices.clear();
	this->skeleton_normal.clear();
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		skeleton_vertices.push_back(mesh.point(*v_it));
		skeleton_normal.push_back(mesh.normal(*v_it));
	}

	mesh.degenerationMeshToLine(skeleton_indices, initial_vertices);

	this->degenerationMeshToLine(0);
}

#pragma endregion

MyMesh::SKHalfedge::SKHalfedge(VertexHandle f, VertexHandle t, float c)
{
	from = f;
	to = t;
	cost = c;
}
