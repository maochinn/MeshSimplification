#include <map>

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
glm::mat4 MyMesh::computeErrorQuadrics(MyMesh::Point point, MyMesh::Normal normal)
{
	float x = point[0];
	float y = point[1];
	float z = point[2];

	//float len = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

	//float a = normal[0] / len;
	//float b = normal[1] / len;
	//float c = normal[2] / len;

	float a = normal[0];
	float b = normal[1];
	float c = normal[2];
	float d = -(a * x + b * y + c * z);

	return glm::mat4(
		a * a, a * b, a * c, a * d,
		a * b, b * b, b * c, b * d,
		a * c, b * c, c * c, c * d,
		a * d, b * d, c * d, d * d);
}

void MyMesh::computeErrorQuadrics()
{
	this->add_property(Q, "Q");

	for (MyMesh::VertexIter v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it)
	{
		MyMesh::Point point = this->point(*v_it);
		MyMesh::Normal normal = this->normal(*v_it);
		
		this->property(Q, *v_it) = this->computeErrorQuadrics(point, normal);
	}
}

void MyMesh::computeError(MyMesh::EdgeHandle e_handle)
{
	glm::mat4 Q1 = this->property(Q, to_vertex_handle(halfedge_handle(e_handle, 0)));
	glm::mat4 Q2 = this->property(Q, from_vertex_handle(halfedge_handle(e_handle, 0)));

	glm::mat4 Q3 = Q1 + Q2;
	
	//glm::mat4 A(Q3);
	//A[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	//float temp = glm::determinant(glm::transpose(A));
	glm::vec4 v;
	//if (abs(temp) < 0.00001f)
	//{
		MyMesh::Point p1 = this->point(to_vertex_handle(halfedge_handle(e_handle, 0)));
		MyMesh::Point p2 = this->point(from_vertex_handle(halfedge_handle(e_handle, 0)));
		MyMesh::Point average = (p1 + p2) * 0.5f;
		v = glm::vec4(average[0], average[1], average[2], 1.0f);
	//}
	//else
	//{
	//	A = glm::inverse(glm::transpose(A));
	//	v = A * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	//	v /= v.w;
	//}
	
	float e = abs(glm::dot(v, Q3 * v));

	this->property(new_Q, e_handle) = Q3;
	this->property(new_v, e_handle) = MyMesh::Point(v.x, v.y, v.z);
	this->property(epsilon, e_handle) = e;
}

void MyMesh::computeError()
{
	this->add_property(new_Q, "new_Q");
	this->add_property(new_v, "new_v");
	this->add_property(epsilon, "epsilon");

	for (MyMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
	{
		computeError(e_it.handle());
	}
}
bool MyMesh::collapse()
{
	EdgeHandle e_handle;
	float min = FLT_MAX;
	for (MyMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
	{
		float e = this->property(epsilon, e_it);
		if (e < min && !status(e_it).deleted() && is_collapse_ok(halfedge_handle(e_it, 0)))
		{
			min = e;
			e_handle = e_it.handle();
		}
	}

	if (min > 1.0)
		return false;

	//print error
	std::cout << min << std::endl;

	HalfedgeHandle he_handle = halfedge_handle(e_handle, 0);
	VertexHandle to = to_vertex_handle(he_handle);

	MyMesh::Point pp = this->property(new_v, e_handle);
	glm::mat4 qq = this->property(new_Q, e_handle);

	this->TriMesh::collapse(he_handle);

	set_point(to, pp);
	this->property(Q, to) = qq;

	//recompute neighbor edge
	for (MyMesh::VertexEdgeIter ve_it = ve_iter(to); ve_it.is_valid(); ++ve_it)
	{
		computeError(ve_it.handle());
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

void MyMesh::simplification()
{
	if (record_vertices.empty())
	{
		int i = 0;
		record();
		for (int j = 0;j<100;j++)
		{
			for (int k = 0; k < 500; k++)
			{
				if (collapse() == false)
				{
					garbage_collection();
					record();
					return;
				}
			}
			garbage_collection();
			record();
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
			mesh.release_face_normals();

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
