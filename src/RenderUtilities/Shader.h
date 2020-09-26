#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#define SHADER_VERSION "#version 430 core\n"

class Shader
{
public:
	GLuint Program;
	enum Type {
		NULL_SHADER = (0),
		VERTEX_SHADER = (1 << 0),
		TESS_CONTROL_SHADER = (1 << 1),
		TESS_EVALUATION_SHADER = (1 << 2),
		GEOMETRY_SHADER = (1 << 3),
		FRAGMENT_SHADER = (1 << 4),
	};
	//DEFINE_ENUM_FLAG_OPERATORS(Type);

	Type type = Shader::NULL_SHADER;
	// Constructor generates the shader on the fly
	Shader(const std::string& vert, const std::string& tesc, const std::string& tese, const std::string& geom, const std::string& frag)
	{
		std::vector<GLuint> shaders;
		if (!vert.empty())
		{
			//if (vert_lib)
			//	code += this->readCode(vert_lib);

			//code += this->readCode(vert);
			//shaders.push_back(this->compileShader(GL_VERTEX_SHADER, code.c_str()));
			shaders.push_back(this->compileShader(GL_VERTEX_SHADER, (SHADER_VERSION + vert).c_str()));
			this->type = (Shader::Type)(this->type | Type::VERTEX_SHADER);
		}
		if (!tesc.empty())
		{
			//std::string code = "#version 430 core\n";
			//code += this->readCode(tesc);
			//shaders.push_back(this->compileShader(GL_TESS_CONTROL_SHADER, code.c_str()));
			shaders.push_back(this->compileShader(GL_TESS_CONTROL_SHADER, (SHADER_VERSION + tesc).c_str()));
			this->type = (Shader::Type)(this->type | Type::TESS_CONTROL_SHADER);
		}
		if (!tese.empty())
		{
			//std::string code = "#version 430 core\n";
			//code += this->readCode(tese);
			//shaders.push_back(this->compileShader(GL_TESS_EVALUATION_SHADER, code.c_str()));
			shaders.push_back(this->compileShader(GL_TESS_EVALUATION_SHADER, (SHADER_VERSION + tese).c_str()));
			this->type = (Shader::Type)(this->type | Type::TESS_EVALUATION_SHADER);
		}
		if (!geom.empty())
		{
			//std::string code = "#version 430 core\n";
			//code += this->readCode(geom);
			//shaders.push_back(this->compileShader(GL_GEOMETRY_SHADER, code.c_str()));
			shaders.push_back(this->compileShader(GL_GEOMETRY_SHADER, (SHADER_VERSION + geom).c_str()));
			this->type = (Shader::Type)(this->type | Type::GEOMETRY_SHADER);
		}
		if (!frag.empty())
		{
			//std::string code = "#version 430 core\n";
			//if (frag_lib)
			//	code += this->readCode(frag_lib);

			//code += this->readCode(frag);
			//shaders.push_back(this->compileShader(GL_FRAGMENT_SHADER, code.c_str()));
			shaders.push_back(this->compileShader(GL_FRAGMENT_SHADER, (SHADER_VERSION + frag).c_str()));
			this->type = (Shader::Type)(this->type | Type::FRAGMENT_SHADER);
		}
		// Shader Program
		GLint success;
		GLchar infoLog[512];
		this->Program = glCreateProgram();

		for (GLuint shader : shaders)
			glAttachShader(this->Program, shader);

		glLinkProgram(this->Program);
		// Print linking errors if any
		glGetProgramiv(this->Program, GL_LINK_STATUS, &success);
		if (!success)
		{
			glGetProgramInfoLog(this->Program, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
		}

		for (GLuint shader : shaders)
			glDeleteShader(shader);
	}
	// Uses the current shader
	void Use()
	{
		glUseProgram(this->Program);
	}
	static std::string readCode(const GLchar* path)
	{
		std::string code;
		std::ifstream shader_file;
		// ensures ifstream objects can throw exceptions:
		shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			// Open files
			shader_file.open(path);
			//if (!vShaderFile)std::cout << vertexPath << " fail to open" << std::endl;
			std::stringstream shader_stream;
			// Read file's buffer contents into streams
			shader_stream << shader_file.rdbuf();
			// close file handlers
			shader_file.close();
			// Convert stream into string
			code = shader_stream.str();
		}
		catch (std::ifstream::failure e)
		{
			std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
			std::cout << path << std::endl;
		}
		return code;
	}
private:
	GLuint compileShader(GLenum shader_type, const char* code)
	{
		GLuint shader_number;
		GLint success;
		GLchar infoLog[512];
		// Vertex Shader
		shader_number = glCreateShader(shader_type);
		glShaderSource(shader_number, 1, &code, NULL);
		glCompileShader(shader_number);
		// Print compile errors if any
		glGetShaderiv(shader_number, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(shader_number, 512, NULL, infoLog);
			if(shader_type == GL_VERTEX_SHADER)
				std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
			else if (shader_type == GL_TESS_CONTROL_SHADER)
				std::cout << "ERROR::SHADER::TESS_CONTROL::COMPILATION_FAILED\n" << infoLog << std::endl;
			else if (shader_type == GL_TESS_EVALUATION_SHADER)
				std::cout << "ERROR::SHADER::TESS_EVALUATION::COMPILATION_FAILED\n" << infoLog << std::endl;
			else if(shader_type == GL_GEOMETRY_SHADER)
				std::cout << "ERROR::SHADER::GEOMETRY::COMPILATION_FAILED\n" << infoLog << std::endl;
			else if (shader_type == GL_FRAGMENT_SHADER)
				std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
		}
		return shader_number;
	}
};

#endif