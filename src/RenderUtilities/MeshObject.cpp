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

	if (abs(det) < 0.0001f)
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
		dQ = dQ.inverse();
		V << dQ(0, 3), dQ(1, 3), dQ(2, 3);
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

	//print error
	//std::cout << min << std::endl;

	HalfedgeHandle he_handle = halfedge_handle(e_handle, 0);
	VertexHandle to = to_vertex_handle(he_handle);

	MyMesh::Point V = this->property(prop_v, e_handle);

	set_point(to, V);

	this->TriMesh::collapse(he_handle);

	//garbage_collection();

	//recompute quadrics & edge errors
	std::set<MyMesh::EdgeHandle> recalc_edges;
	this->computeErrorQuadrics(to);
	for (MyMesh::VertexVertexIter vv_it = vv_iter(to); vv_it.is_valid(); ++vv_it)
	{
		if (!status(vv_it).deleted()) {

			this->computeErrorQuadrics(vv_it);

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

double MyMesh::computeWeight(MyMesh::HalfedgeHandle& heh)
{
	GLdouble alpha, beta, weight;
	MyMesh::Point pFrom = point(from_vertex_handle(heh));
	MyMesh::Point pTo = point(to_vertex_handle(heh));
	MyMesh::Point p1 = point(opposite_vh(heh));
	MyMesh::Point p2 = point(opposite_he_opposite_vh(heh));

	OpenMesh::Vec3d v1 = (OpenMesh::Vec3d)(p1 - pFrom).normalize();
	OpenMesh::Vec3d v2 = (OpenMesh::Vec3d)(p1 - pTo).normalize();
	alpha = std::acos(clamp(OpenMesh::dot(v1, v2), -0.999, 0.999));

	v1 = (OpenMesh::Vec3d)(p2 - pFrom).normalize();
	v2 = (OpenMesh::Vec3d)(p2 - pTo).normalize();
	beta = std::acos(clamp(OpenMesh::dot(v1, v2), -0.999, 0.999));

	weight = std::cos(alpha) / std::sin(alpha) + std::cos(beta) / std::sin(beta);
	assert(!isnan(weight));
	return weight;
}
void MyMesh::degenerateLeastSquareMesh(std::vector<MyMesh::Point>& points, double W0_H, double W0_L)
{
	const int N(n_vertices());

	// using all vertices to be control points
	Eigen::SparseMatrix<double> A(N + N, N);
	Eigen::SparseMatrix<double> b(N + N, 3);
	std::vector<Eigen::Triplet<double>> triplet_list_A, triplet_list_b;

	std::map<VertexHandle, int> vertices;
	std::vector<double> A0(N), At(N);
	int idx(0);
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it, ++idx)
	{
		vertices.insert(std::pair<VertexHandle, int>(v_it.handle(), idx));
		//for (auto it = faces_begin(); it != faces_end(); ++it)
		for (MyMesh::VertexFaceIter vf_it = vf_iter(v_it); vf_it.is_valid(); ++vf_it)
		{
			A0[idx] = At[idx] = calc_face_area(vf_it);
		}
	}

	//double W0_H = 1.0;
	std::vector<double> W_H(N, W0_H);

	double W_L(0.0);
	for (auto it = faces_begin(); it != faces_end(); ++it)
	{
		W_L += calc_face_area(it);
	}
	//W_L = 0.1 * sqrt(W_L/(double)n_faces());
	W_L = W0_L;

	for (int t(0); t < 1; t++)
	{
		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			// Laplacian
			int i = it->second;	//col
			int j = it->second;	//row

			double w_ij = 0.0;
			for (MyMesh::VertexOHalfedgeIter voh_it = voh_iter(it->first); voh_it.is_valid(); ++voh_it)
			{
				double w_ik = computeWeight(voh_it.handle());
				w_ij += -w_ik;
			}
			double temp = W_L * w_ij;
			if (temp != temp)
			{
				puts("!!!");
				std::cout << W_L << "*" << w_ij << std::endl;
			}

			triplet_list_A.push_back(Eigen::Triplet<double>(j, i, W_L * w_ij));


			for (MyMesh::VertexOHalfedgeIter voh_it = voh_iter(it->first); voh_it.is_valid(); ++voh_it)
			{
				i = vertices[to_vertex_handle(voh_it)];
				w_ij = computeWeight(voh_it.handle());
				triplet_list_A.push_back(Eigen::Triplet<double>(j, i, W_L * w_ij));

				temp = W_L * w_ij;
				if (temp != temp)
				{
					puts("???");
					std::cout << W_L << "*" << w_ij << std::endl;
				}

			}

			// Constraint
			i = it->second;
			j = N + it->second;
			triplet_list_A.push_back(Eigen::Triplet<double>(j, i, W_H[i]));

			MyMesh::Point vi = point(it->first);
			triplet_list_b.push_back(Eigen::Triplet<double>(j, 0, W_H[i] * vi[0]));
			triplet_list_b.push_back(Eigen::Triplet<double>(j, 1, W_H[i] * vi[1]));
			triplet_list_b.push_back(Eigen::Triplet<double>(j, 2, W_H[i] * vi[2]));
		}

		//fullfill A and b
		A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());
		b.setFromTriplets(triplet_list_b.begin(), triplet_list_b.end());

		Eigen::SparseMatrix<double> ATA = A.transpose() * A;
		Eigen::SparseMatrix<double> ATb = A.transpose() * b;

		//
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(ATA);
		Eigen::MatrixXd x = solver.solve(ATb);
		std::cout << x.row(0).col(0).value() << std::endl;

		//update
		W_L *= 2.0;
		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			int idx = it->second;
			float xx = x.row(idx).col(0).value();
			float yy = x.row(idx).col(1).value();
			float zz = x.row(idx).col(2).value();
			//if (xx != xx || yy != yy || zz != zz)
			//{
			//	std::cout << xx << " " << yy << " " << zz << std::endl;
			//}
			set_point(it->first, MyMesh::Point(xx, yy, zz));
		}

		for (auto it = vertices.begin(); it != vertices.end(); it++)
		{
			for (MyMesh::VertexFaceIter vf_it = vf_iter(it->first); vf_it.is_valid(); ++vf_it)
			{
				At[idx] = calc_face_area(vf_it);
			}

			W_H[idx] = W0_H/* * sqrt(A0[idx] / At[idx])*/;
		}
	}

	//
	points.reserve(N);
	for (auto it = vertices.begin(); it != vertices.end(); it++)
	{
		int idx = it->second;
		//points[idx][0] = x.row(idx).col(0).value();
		//points[idx][1] = x.row(idx).col(1).value();
		//points[idx][2] = x.row(idx).col(2).value();
		points[idx] = point(it->first);
	}
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
	glDrawElements(GL_TRIANGLES, this->vao.element_amount, GL_UNSIGNED_INT, 0);
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
	std::vector<MyMesh::Point>& vertices, std::vector<MyMesh::Normal>& normals, std::vector<unsigned int>& indices)
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

void GLMesh::degenerateLeastSquareMesh()
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

	mesh.degenerateLeastSquareMesh(vertices, 1.0, 10.0);

	LoadToShader(vertices, normals, indices);
}

#pragma endregion
