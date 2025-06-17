#pragma once

#include <glad/glad.h>

#include <string>

class Shader {
 public:
  Shader(std::string const & vertex_shader_filename, std::string const & fragment_shader_filename);
  ~Shader();

  void Activate() const;

 private:
  GLuint program_id_;
  static std::string Read(std::string const & filename);
};
