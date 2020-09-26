#pragma once
#include <opencv2\opencv.hpp>

#include "BufferObject.h"
#include "Shader.h"

class CubeMap
{
public:
	/*load 6 face image to create*/
	CubeMap(Shader shader,
		const char* right,
		const char* left,
		const char* top,
		const char* down,
		const char* back,
		const char* front)
		:shader(shader)
	{
		//load texture
		this->id = this->loadTexture(right, left, top, down, back, front);

		this->vao = this->generateVAO();
	}
	/*load already created cubemap texture*/
	CubeMap(Shader shader, GLuint texture_id)
		:shader(shader)
	{

		this->id = texture_id;

		this->vao = this->generateVAO();
	}
	~CubeMap()
	{
		
	}
	void bind(int bind_unit)
	{
		glActiveTexture(GL_TEXTURE0 + bind_unit);
		glBindTexture(GL_TEXTURE_CUBE_MAP, this->id);
	}
	void render()
	{
		this->shader.Use();

		this->bind(0);
		glUniform1i(glGetUniformLocation(this->shader.Program, "u_material.texture_cubemap"), 0);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);  // Change depth function so depth test passes when values are equal to depth buffer's content
		glBindVertexArray(this->vao.vao);
		glDrawArrays(GL_TRIANGLES, 0, this->vao.count);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS); // Set depth function back to default
	}
private:
	VAO vao;
	Shader shader;
	GLuint id;	//texture id
	GLuint loadTexture(
		const char* right,
		const char* left,
		const char* top,
		const char* down,
		const char* back,
		const char* front)
	{
		unsigned int id;
		glGenTextures(1, &id);
		glBindTexture(GL_TEXTURE_CUBE_MAP, id);
		//6 face
		for (int i = 0; i < 6; i++)
		{
			cv::Mat img;
			if(i == 0)
				img = cv::imread(right, cv::IMREAD_COLOR);
			else if(i == 1)
				img = cv::imread(left, cv::IMREAD_COLOR);
			else if (i == 2)
				img = cv::imread(top, cv::IMREAD_COLOR);
			else if (i == 3)
				img = cv::imread(down, cv::IMREAD_COLOR);
			else if (i == 4)
				img = cv::imread(back, cv::IMREAD_COLOR);
			else if (i == 5)
				img = cv::imread(front, cv::IMREAD_COLOR);

			if (img.empty())
			{
				puts("CUBEMAP::ERROR::LOAD");
				return 0;
			}
			//cv::cvtColor(img, img, CV_BGR2RGB);

			if (img.type() == CV_8UC3)
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB8, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
			else if (img.type() == CV_8UC4)
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA8, img.cols, img.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, img.data);
			glBindTexture(GL_TEXTURE_2D, 0);

			img.release();
		}

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);


		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
		return id;
	}
	VAO generateVAO()
	{
		float skyboxVertices[] = {
			// Positions          
			-1.0f,  1.0f, -1.0f,
			-1.0f, -1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f,  1.0f, -1.0f,
			-1.0f,  1.0f, -1.0f,

			-1.0f, -1.0f,  1.0f,
			-1.0f, -1.0f, -1.0f,
			-1.0f,  1.0f, -1.0f,
			-1.0f,  1.0f, -1.0f,
			-1.0f,  1.0f,  1.0f,
			-1.0f, -1.0f,  1.0f,

			1.0f, -1.0f, -1.0f,
			1.0f, -1.0f,  1.0f,
			1.0f,  1.0f,  1.0f,
			1.0f,  1.0f,  1.0f,
			1.0f,  1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,

			-1.0f, -1.0f,  1.0f,
			-1.0f,  1.0f,  1.0f,
			1.0f,  1.0f,  1.0f,
			1.0f,  1.0f,  1.0f,
			1.0f, -1.0f,  1.0f,
			-1.0f, -1.0f,  1.0f,

			-1.0f,  1.0f, -1.0f,
			1.0f,  1.0f, -1.0f,
			1.0f,  1.0f,  1.0f,
			1.0f,  1.0f,  1.0f,
			-1.0f,  1.0f,  1.0f,
			-1.0f,  1.0f, -1.0f,

			-1.0f, -1.0f, -1.0f,
			-1.0f, -1.0f,  1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			-1.0f, -1.0f,  1.0f,
			1.0f, -1.0f,  1.0f
		};

		VAO vao;

		vao.count = 36;
		glGenVertexArrays(1, &vao.vao);
		glGenBuffers(1, vao.vbo);
		glBindVertexArray(vao.vao);
		glBindBuffer(GL_ARRAY_BUFFER, vao.vbo[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
		glBindVertexArray(0);

		return vao;
	}
};