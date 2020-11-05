#include <map>
#include <set>
#include "MeshObject.h"
struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

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

		LoadToShader();
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

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	vertices.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));

		MyMesh::Point p = mesh.point(*v_it);
	}

	std::vector<MyMesh::Normal> normals;
	normals.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		normals.push_back(mesh.normal(*v_it));
	}

	std::vector<unsigned int> indices;
	indices.reserve(mesh.n_faces() * 3);
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
	{
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
			indices.push_back(fv_it->idx());
		}
	}
	std::cout << "vertex number: " << mesh.n_vertices() << std::endl;
	std::cout << "edge number: " << mesh.n_edges() << std::endl;
	std::cout << "half edge number: " << mesh.n_halfedges() << std::endl;
	std::cout << "face number: " << mesh.n_faces() << std::endl;

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
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), &indices[0], GL_STATIC_DRAW);

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
	//int i(0);
	//for (MyMesh::HalfedgeIter it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it, i++) {
	//	if (i % 4 == 0 && mesh.is_collapse_ok(*it))
	//	{
	//		MyMesh::VertexHandle from = mesh.from_vertex_handle(*it);
	//		MyMesh::VertexHandle to = mesh.to_vertex_handle(*it);
	//		MyMesh::Point average = (mesh.point(from) + mesh.point(to)) * 0.5f;
	//		
	//		// Collapse edge
	//		mesh.collapse(*it);

	//		mesh.set_point(to, average);
	//	}
	//}
	//mesh.garbage_collection();


	if (mesh.record_vertices.empty())
	{
		mesh.simplification();
	}

	int idx = (int)((1.0f - ratio) * (mesh.record_vertices.size()-1));
	//idx = 1;
	//reload
	//LoadToShader();

	//std::cout << mesh.record_indices.size() << std::endl;

	this->vao.element_amount = mesh.record_indices[idx].size();

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * mesh.record_vertices[idx].size(), &mesh.record_vertices[idx][0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->vao.vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * mesh.record_normals[idx].size(), &mesh.record_normals[idx][0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vao.ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * mesh.record_indices[idx].size(), &mesh.record_indices[idx][0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLMesh::collispe()
{
	//for (MyMesh::HalfedgeIter it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it) {
	//	if (mesh.is_collapse_ok(*it))
	//	{
	//		MyMesh::VertexHandle from = mesh.from_vertex_handle(*it);
	//		MyMesh::VertexHandle to = mesh.to_vertex_handle(*it);
	//		MyMesh::Point average = (mesh.point(from) + mesh.point(to)) * 0.5f;

	//		// Collapse edge
	//		mesh.collapse(*it);

	//		mesh.set_point(to, average);

	//		break;
	//	}
	//}
	mesh.simplification();

	LoadToShader();
}
void GLMesh::uncollispe()
{

}

#pragma endregion
