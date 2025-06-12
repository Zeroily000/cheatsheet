// clang-format off
/**
 * Be sure to include GLAD before GLFW. The include file for GLAD includes the required OpenGL
 * headers behind the scenes (like GL/gl.h) so be sure to include GLAD before other header files
 * that require OpenGL (like GLFW).
 */
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <cstdlib>
#include <iostream>

namespace {

constexpr int kDefaultWidth{800};
constexpr int kDefaultHeight{600};

char const * const kVertexShaderSource{
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
    "}\0",
};
char const * const kFragmentShaderSource{
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
    "}\n\0",
};

constexpr float kVertices[]{
    -0.5f, -0.5f, 0.0f,  // left
    0.5f,  -0.5f, 0.0f,  // right
    0.0f,  0.5f,  0.0f   // top
};

/**
 * @brief Whenever the window size changed (by OS or user resize) this callback function executes.
 */
void framebufferSizeCallback(GLFWwindow * const window, int const width, int const height) {
  // make sure the viewport matches the new window dimensions; note that width and
  // height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

/**
 * @brief process all input: query GLFW whether relevant keys are pressed/released this frame and
 * react accordingly
 */
void processInput(GLFWwindow * const window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  }
}

}  // namespace

int main() {
  // Initialize and configure
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Window creation
  GLFWwindow * const window{
      glfwCreateWindow(kDefaultWidth, kDefaultHeight, "LearnOpenGL", nullptr, nullptr),
  };
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return EXIT_FAILURE;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

  // Load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return EXIT_FAILURE;
  }

  // Build and compile shader program
  // Vertex shader
  unsigned int const vertex_shader{glCreateShader(GL_VERTEX_SHADER)};
  glShaderSource(vertex_shader, 1, &kVertexShaderSource, nullptr);
  glCompileShader(vertex_shader);
  {
    int success{false};
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char info_log[512];
      glGetShaderInfoLog(vertex_shader, 512, nullptr, info_log);
      std::cout << "Compiling vertex shader failed: " << info_log;
    }
  }

  // Fragment shader
  unsigned int const fragment_shader{glCreateShader(GL_FRAGMENT_SHADER)};
  glShaderSource(fragment_shader, 1, &kFragmentShaderSource, nullptr);
  glCompileShader(fragment_shader);
  {
    int success{false};
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char info_log[512];
      glGetShaderInfoLog(fragment_shader, 512, nullptr, info_log);
      std::cout << "Compiling fragment shader failed: " << info_log;
    }
  }

  // Link shaders
  unsigned int const shader_program{glCreateProgram()};
  glAttachShader(shader_program, vertex_shader);
  glAttachShader(shader_program, fragment_shader);
  glLinkProgram(shader_program);
  {
    int success{false};
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
    if (!success) {
      char info_log[512];
      glGetProgramInfoLog(shader_program, 512, nullptr, info_log);
      std::cout << "Linking shaders failed: " << info_log;
    }
  }

  unsigned int vao;
  glGenVertexArrays(1, &vao);
  unsigned int vbo;
  glGenBuffers(1, &vbo);
  // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure
  // vertex attributes(s).
  glBindVertexArray(vao);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(kVertices), kVertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex
  // attribute's bound vertex buffer object so afterwards we can safely unbind
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but
  // this rarely happens. Modifying other VAOs requires a call to glBindVertexArray anyways so we
  // generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
  glBindVertexArray(0);

  // Render loop
  while (!glfwWindowShouldClose(window)) {
    // Input
    processInput(window);

    // Render
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // draw our first triangle
    glUseProgram(shader_program);
    glBindVertexArray(vao);  // seeing as we only have a single VAO there's no need to bind it every
                             // time, but we'll do so to keep things a bit more organized
    glDrawArrays(GL_TRIANGLES, 0, 3);
    // glBindVertexArray(0); // no need to unbind it every time

    // Swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
  glDeleteProgram(shader_program);

  // Terminate, clearing all previously allocated GLFW resources.
  glfwTerminate();
  return EXIT_SUCCESS;
}
