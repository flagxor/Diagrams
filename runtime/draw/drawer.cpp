// Copyright 2015 Native Client Authors.

#include <iostream>
#include <vector>
#include <string>

#include "draw/drawer.h"
#include "draw/camera.h"
#include "geometry/geometry2.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"

namespace {

const GLuint kVertDim = 4;

const char kFragShaderSource[] =
    "precision mediump float;\n"
    "varying vec4 v_color;\n"
    "varying vec2 tex_coord;\n"
    "uniform sampler2D tex_data;\n"
    "void main() {\n"
    "  vec4 alpha = vec4(1.0, 1.0, 1.0, texture2D(tex_data, tex_coord).a);\n"
    "  gl_FragColor = v_color * alpha;\n"
    "}\n";

const char kVertShaderSource[] =
    "uniform mat4 u_mvp;\n"
    "attribute vec4 position;\n"
    "attribute vec4 color;\n"
    "varying vec4 v_color;\n"
    "varying vec2 tex_coord;\n"
    "void main() {\n"
    "  gl_Position = u_mvp * vec4(position.xy, 0.0, 1.0);\n"
    "  tex_coord = position.zw;\n"
    "  v_color = vec4(color.xyz, 1.0);\n"
    "}\n";

void ShaderErrorHandler(GLuint shader_id) {
  GLint log_size;
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> compileLog(log_size);
  glGetShaderInfoLog(shader_id, log_size, nullptr, compileLog.data());
  std::cerr << "Compile Error: " << compileLog.data() << std::endl;
}

void ProgarmErrorHanlder(GLuint program_id) {
  GLint log_size;
  glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> errorMessage(log_size);
  glGetProgramInfoLog(program_id, log_size, nullptr, errorMessage.data());
  std::cerr << "Linking Error: " << errorMessage.data() << std::endl;
}

GLuint CompileShaderFromSource(const char* data, GLenum type) {
  GLuint shader_id = glCreateShader(type);
  glShaderSource(shader_id, 1, &data, nullptr);
  glCompileShader(shader_id);
  GLint result = GL_FALSE;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (result != GL_TRUE) {
    ShaderErrorHandler(shader_id);
    return 0;
  }
  return shader_id;
}

GLuint CompileShaderFromFile(const char* fname, GLenum type) {
  std::string shader_text = diagrammar::Stringify(fname);
  const char* shader_text_cstr = shader_text.c_str();
  return CompileShaderFromSource(shader_text_cstr, type);
}

/*
void RenderString(FT_Face fc, std::string s, float x, float y, float sx,
                    float sy) {
  for (auto ch : s) {
    if (FT_Load_Char(fc, ch, FT_LOAD_RENDER)) continue;
    FT_GlyphSlot glyph = fc->glyph;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, glyph->bitmap.width,
                 glyph->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE,
                 glyph->bitmap.buffer);
    // now we have added a texture
    // let us create a box, the x and y are cursor position
    // sx and sy are scale in each direction
    GLfloat xpos = x + glyph->bitmap_left * sx;
    GLfloat ypos = y + glyph->bitmap_top * sy;
    GLfloat w = glyph->bitmap.width * sx;
    GLfloat h = glyph->bitmap.rows * sy;
    GLfloat char_box[16] = {xpos, ypos,     0, 0, xpos + w, ypos,     1, 0,
                            xpos, ypos - h, 0, 1, xpos + w, ypos - h, 1, 1};
    glBufferData(GL_ARRAY_BUFFER, sizeof(char_box), char_box, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    x += (glyph->advance.x >> 6) * sx;
    y += (glyph->advance.y >> 6) * sy;
  }
}
*/

}  // namespace

namespace diagrammar {

GLProgram LoadDefaultGLProgram() {
  GLuint vert_shader_id =
      CompileShaderFromSource(kVertShaderSource, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(kFragShaderSource, GL_FRAGMENT_SHADER);
  // now we can link the program

  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vert_shader_id);
  glAttachShader(program_id, frag_shader_id);
  glLinkProgram(program_id);
  GLint result;
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  if (result != GL_TRUE) {
    ProgarmErrorHanlder(program_id);
  }
  glDeleteShader(vert_shader_id);
  glDeleteShader(frag_shader_id);

  GLProgram program;
  program.texture_loc = glGetUniformLocation(program_id, "tex_data");
  program.u_mvp_loc = glGetUniformLocation(program_id, "u_mvp");
  program.color_loc = glGetAttribLocation(program_id, "color");
  program.vertex_loc = glGetAttribLocation(program_id, "position");
  program.program_id = program_id;
  return program;
}

NodePathDrawer::NodePathDrawer(Node* node) {
  node_ = node;
  GenBuffers();
}

NodePathDrawer::~NodePathDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
}

void NodePathDrawer::GenPathBuffer(const Path& polyline, bool is_closed) {
  GLuint vert_vbo;
  glGenBuffers(1, &vert_vbo);
  vertex_buffer_.emplace_back(vert_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
  size_t num_vertices = polyline.size();

  std::vector<GLfloat> vertices(num_vertices * kVertDim);
  for (size_t j = 0; j < polyline.size(); ++j) {
    vertices[kVertDim * j + 0] = polyline[j](0);
    vertices[kVertDim * j + 1] = polyline[j](1);
    vertices[kVertDim * j + 2] = 0;
    vertices[kVertDim * j + 3] = 1;
  }
  if (is_closed) {
    vertices.emplace_back(polyline[0](0));
    vertices.emplace_back(polyline[0](1));
    vertices.emplace_back(0);
    vertices.emplace_back(0);
    num_vertices++;
  }
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
               vertices.data(), GL_STATIC_DRAW);

  // count for closed loops
  vertex_size_.emplace_back(num_vertices);

  GLuint color_vbo;
  glGenBuffers(1, &color_vbo);
  vertex_color_buffer_.emplace_back(color_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, color_vbo);

  std::vector<GLfloat> colors(vertices.size());
  for (size_t j = 0; j < num_vertices; ++j) {
    colors[kVertDim * j + 0] = 1;
    colors[kVertDim * j + 1] = 1;
    colors[kVertDim * j + 2] = 1;
    colors[kVertDim * j + 3] = 1;
  }
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);
}

void NodePathDrawer::GenBuffers() {
  assert(node_ != nullptr);

  // All closed path
  for (auto& polygon : node_->polygons) {
    unsigned num_paths = 1 + polygon.holes.size();
    for (size_t pa_idx = 0; pa_idx < num_paths; ++pa_idx) {
      const std::vector<Vector2f>& polyline =
          pa_idx == 0 ? polygon.path : polygon.holes[pa_idx - 1];
      GenPathBuffer(polyline, true);
    }
  }

  // All open path
  for (auto& path : node_->paths) {
    GenPathBuffer(path, false);
  }
}

void NodePathDrawer::Draw(GLProgram program, Camera* camera) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Matrix4f u_mvp(Matrix4f::Identity());
    u_mvp.topLeftCorner<2, 2>() = node_->frame.GetRotationMatrix();
    u_mvp.col(3).head<2>() = node_->frame.GetTranslation();
    u_mvp = camera->GetViewProjection() * u_mvp;
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

    glDrawArrays(GL_LINE_STRIP, 0, vertex_size_[i]);
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(program.vertex_loc);
  glDisableVertexAttribArray(program.color_loc);
}

NodePolyDrawer::NodePolyDrawer(Node* node) {
  node_ = node;
  GenBuffers();
}

NodePolyDrawer::~NodePolyDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
  glDeleteBuffers(index_buffer_.size(), index_buffer_.data());
}

void NodePolyDrawer::GenBuffers() {
  assert(node_ != nullptr);

  // All closed path
  for (auto& polygon : node_->polygons) {
    auto mesh = TriangulatePolygon(polygon);
    GenTriangleBuffer(mesh);
  }

  // All open path
  for (auto& path : node_->paths) {
    auto mesh = TriangulatePolyline(path, 1.5);
    GenTriangleBuffer(mesh);
  }
}

// TODO(tingnan) fix rendering
void NodePolyDrawer::GenTriangleBuffer(const TriangleMesh& mesh) {
  size_t num_vertices = mesh.vertices.size();
  vertex_size_.emplace_back(num_vertices);

  GLuint v_buffer;
  glGenBuffers(1, &v_buffer);
  vertex_buffer_.emplace_back(v_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, v_buffer);

  // the vertes array
  std::vector<GLfloat> vert_array;
  vert_array.reserve(num_vertices * kVertDim);
  for (auto& vt : mesh.vertices) {
    vert_array.emplace_back(vt(0));
    vert_array.emplace_back(vt(1));
    vert_array.emplace_back(0);
    vert_array.emplace_back(0);
  }
  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);

  GLuint i_buffer;
  glGenBuffers(1, &i_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, i_buffer);
  index_buffer_.emplace_back(i_buffer);
  std::vector<GLuint> indices;
  indices.reserve(mesh.faces.size() * 3);
  for (auto& face : mesh.faces) {
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      indices.emplace_back(face[vt_idx]);
    }
  }
  index_size_.emplace_back(indices.size());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               indices.data(), GL_STATIC_DRAW);

  GLuint c_buffer;
  glGenBuffers(1, &c_buffer);
  vertex_color_buffer_.emplace_back(c_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, c_buffer);
  std::vector<GLfloat> colors(num_vertices * kVertDim);
  // each triangle is labeled with r g b
  for (size_t cid = 0; cid < num_vertices; ++cid) {
    if (cid % 3 == 0) {
      colors[kVertDim * cid + 0] = 1;
      colors[kVertDim * cid + 1] = 1;
      colors[kVertDim * cid + 2] = 0;
      colors[kVertDim * cid + 3] = 1;
    }
    if (cid % 3 == 1) {
      colors[kVertDim * cid + 0] = 1;
      colors[kVertDim * cid + 1] = 0;
      colors[kVertDim * cid + 2] = 1;
      colors[kVertDim * cid + 3] = 1;
    }
    if (cid % 3 == 2) {
      colors[kVertDim * cid + 0] = 0;
      colors[kVertDim * cid + 1] = 1;
      colors[kVertDim * cid + 2] = 1;
      colors[kVertDim * cid + 3] = 1;
    }
  }
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);
}

void NodePolyDrawer::Draw(GLProgram program, Camera* camera) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Matrix4f u_mvp(Matrix4f::Identity());
    u_mvp.topLeftCorner<2, 2>() = node_->frame.GetRotationMatrix();
    u_mvp.col(3).head<2>() = node_->frame.GetTranslation();
    u_mvp = camera->GetViewProjection() * u_mvp;
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_[i]);
    glDrawElements(GL_TRIANGLES, index_size_[i], GL_UNSIGNED_INT, 0);
  }
  glDisableVertexAttribArray(program.vertex_loc);
  glDisableVertexAttribArray(program.color_loc);
}

// move back to header
template <class DrawerType>
Canvas<DrawerType>::Canvas(GLProgram program, Camera* camera)
    : program_(program), camera_(camera) {}

template <class DrawerType>
void Canvas<DrawerType>::AddNode(Node* node) {
  drawers_[node->id] = make_unique<DrawerType>(node);
}

template <class DrawerType>
void Canvas<DrawerType>::RemoveNodeByID(int id) {
  if (drawers_.find(id) != drawers_.end()) {
    drawers_.erase(id);
  }
}

template <class DrawerType>
void Canvas<DrawerType>::Draw() {
  glUseProgram(program_.program_id);
  for (auto itr = drawers_.begin(); itr != drawers_.end(); ++itr) {
    itr->second->Draw(program_, camera_);
  }
}

// to prevent linking error
template class Canvas<NodePathDrawer>;
template class Canvas<NodePolyDrawer>;

}  // namespace diagrammar
