#include "examples/opengl/shader.hpp"

#include <GLFW/glfw3.h>

#include <fstream>
#include <iostream>
#include <iterator>

Shader::Shader(std::string const & vertex_shader_filename,
               std::string const & fragment_shader_filename) {
  // Vertex shader
  std::string const vertex_shader_code{Read(vertex_shader_filename)};
  char const * const vertex_shader_code_cstr{vertex_shader_code.c_str()};
  GLuint const vertex_shader_id{glCreateShader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader_id, 1, &vertex_shader_code_cstr, nullptr);
  glCompileShader(vertex_shader_id);
  {
    GLint success{0};
    glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &success);
    if (success == 0) {
      char info_log[512];
      glGetShaderInfoLog(vertex_shader_id, 512, nullptr, info_log);
      std::cout << "Compiling vertex shader failed: " << info_log;
    }
  }

  // Fragment shader
  std::string const fragment_shader_code{Read(fragment_shader_filename)};
  char const * const fragment_shader_code_cstr{fragment_shader_code.c_str()};
  GLuint const fragment_shader_id{glCreateShader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader_id, 1, &fragment_shader_code_cstr, nullptr);
  glCompileShader(fragment_shader_id);
  {
    GLint success{0};
    glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &success);
    if (success == 0) {
      char info_log[512];
      glGetShaderInfoLog(fragment_shader_id, 512, nullptr, info_log);
      std::cout << "Compiling fragment shader failed: " << info_log;
    }
  }

  // Shader program
  program_id_ = glCreateProgram();
  glAttachShader(program_id_, vertex_shader_id);
  glAttachShader(program_id_, fragment_shader_id);
  glLinkProgram(program_id_);
  {
    GLint success{0};
    glGetProgramiv(program_id_, GL_LINK_STATUS, &success);
    if (!success) {
      char info_log[512];
      glGetProgramInfoLog(program_id_, 512, nullptr, info_log);
      std::cout << "Linking shaders failed: " << info_log;
    }
  }

  // Delete the shaders as they're linked into the program now and no longer necessary
  glDeleteShader(vertex_shader_id);
  glDeleteShader(fragment_shader_id);
}

Shader::~Shader() { glDeleteProgram(program_id_); }

void Shader::Activate() const { glUseProgram(program_id_); }

std::string Shader::Read(std::string const & filename) {
  std::ifstream ifs{filename};
  return {std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
}
