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

#include "examples/opengl/shader.hpp"

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
char const * const kFragmentShaderSource[]{
    {
        "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
        "}\n\0",
    },
    {
        "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(1.0f, 1.0f, 0.0f, 1.0f);\n"
        "}\n\0",
    },
};

// clang-format off
constexpr float kVertices[][18]{
  // {
  //   -.9f, .9f, 0.f,  // top left
  //   -.9f, 0.f, 0.f,  // bottom left
  //    0.f, 0.f, 0.f,  // bottom right
  //    0.f, .9f, 0.f,  // top right
  // },
  // {
  //   0.f,  0.f, 0.f,  // top left
  //   0.f, -.9f, 0.f,  // bottom left
  //   .9f, -.9f, 0.f,  // bottom right
  //   .9f,  0.f, 0.f,  // top right
  // },
  {
    // positions         // colors
     0.5f, -0.5f, 0.0f,  1.0f, 0.0f, 0.0f,  // bottom right
    -0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f,  // bottom left
     0.0f,  0.5f, 0.0f,  0.0f, 0.0f, 1.0f   // top
  },
};

constexpr unsigned int kIndices[][6]{
  {
    0, 1, 3,  // first Triangle
    1, 2, 3,  // second Triangle
  },
  {
    0, 1, 3,  // first Triangle
    1, 2, 3,  // second Triangle
  }
};
// clang-format on

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
  if (glfwInit() != GLFW_TRUE) {
    std::cout << "Failed to initialize GLFW" << std::endl;
    return EXIT_FAILURE;
  }
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
  if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return EXIT_FAILURE;
  }

  Shader const shader{"examples/opengl/shaders/default.vs", "examples/opengl/shaders/default.fs"};
  // // Build and compile shader program
  // // Vertex shader
  // GLuint const vertex_shader{glCreateShader(GL_VERTEX_SHADER)};
  // glShaderSource(vertex_shader, 1, &kVertexShaderSource, nullptr);
  // glCompileShader(vertex_shader);
  // {
  //   int success{false};
  //   glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
  //   if (!success) {
  //     char info_log[512];
  //     glGetShaderInfoLog(vertex_shader, 512, nullptr, info_log);
  //     std::cout << "Compiling vertex shader failed: " << info_log;
  //   }
  // }

  // // Fragment shader
  // constexpr std::size_t kNumShaders{
  //     sizeof(kFragmentShaderSource) / sizeof(kFragmentShaderSource[0]),
  // };
  // GLuint fragment_shader[kNumShaders];
  // for (std::size_t i{0}; i < kNumShaders; ++i) {
  //   fragment_shader[i] = glCreateShader(GL_FRAGMENT_SHADER);
  //   glShaderSource(fragment_shader[i], 1, &kFragmentShaderSource[i], nullptr);
  //   glCompileShader(fragment_shader[i]);
  //   int success{false};
  //   glGetShaderiv(fragment_shader[i], GL_COMPILE_STATUS, &success);
  //   if (!success) {
  //     char info_log[512];
  //     glGetShaderInfoLog(fragment_shader[i], 512, nullptr, info_log);
  //     std::cout << "Compiling fragment shader failed: " << info_log;
  //   }
  // }

  // // Link shaders
  // GLuint shader_program[kNumShaders];
  // for (std::size_t i{0}; i < kNumShaders; ++i) {
  //   shader_program[i] = glCreateProgram();
  //   glAttachShader(shader_program[i], vertex_shader);
  //   glAttachShader(shader_program[i], fragment_shader[i]);
  //   glLinkProgram(shader_program[i]);
  //   int success{false};
  //   glGetProgramiv(shader_program[i], GL_LINK_STATUS, &success);
  //   if (!success) {
  //     char info_log[512];
  //     glGetProgramInfoLog(shader_program[i], 512, nullptr, info_log);
  //     std::cout << "Linking shaders failed: " << info_log;
  //   }
  // }

  // constexpr std::size_t kNumBuffers{sizeof(kVertices) / sizeof(kVertices[0])};
  constexpr std::size_t kNumBuffers{1};
  GLuint vao[kNumBuffers];
  glGenVertexArrays(kNumBuffers, vao);
  GLuint vbo[kNumBuffers];
  glGenBuffers(kNumBuffers, vbo);
  // GLuint ebo[kNumBuffers];
  // glGenBuffers(kNumBuffers, ebo);

  for (std::size_t i{0}; i < kNumBuffers; ++i) {
    // Bind the Vertex Array Object
    glBindVertexArray(vao[i]);

    // Bind and set the Vertex Buffer Object
    glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kVertices[i]), kVertices[i], GL_STATIC_DRAW);

    // Bind and set the Element Buffer Object
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo[i]);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kIndices[i]), kIndices[i], GL_STATIC_DRAW);

    // Configure vertex attributes
    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    // glEnableVertexAttribArray(0);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex
    // attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but
    // this rarely happens. Modifying other VAOs requires a call to glBindVertexArray anyways so we
    // generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
  }

  // Draw in wireframe polygons.
  // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // Render loop
  while (glfwWindowShouldClose(window) == 0) {
    // Input
    processInput(window);

    // Render
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    shader.Activate();
    for (std::size_t i{0}; i < kNumBuffers; ++i) {
      // glUseProgram(shader_program[i]);
      glBindVertexArray(vao[i]);  // seeing as we only have a single VAO there's no need to bind it
                                  // every time, but we'll do so to keep things a bit more organized
      glDrawArrays(GL_TRIANGLES, 0, 3);
      // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
      // glBindVertexArray(0); // no need to unbind it every time
    }

    // Swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Optional: de-allocate all resources once they've outlived their purpose:
  glDeleteVertexArrays(kNumBuffers, vao);
  glDeleteBuffers(kNumBuffers, vbo);
  // glDeleteBuffers(kNumBuffers, ebo);

  // Terminate, clearing all previously allocated GLFW resources.
  glfwTerminate();
  return EXIT_SUCCESS;
}
