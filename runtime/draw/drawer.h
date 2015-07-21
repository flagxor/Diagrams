// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_DRAW_DRAWER_H_
#define RUNTIME_DRAW_DRAWER_H_

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GLES2/gl2.h>
#endif

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "physics/node.h"

namespace diagrammar {

class Camera;

struct GLProgram {
  // the user provided model_view_projection matrix
  GLuint u_mvp_loc;
  GLuint color_loc;
  GLuint vertex_loc;
  GLuint texture_loc;
  GLuint program_id;
};

GLProgram LoadDefaultGLProgram();

class NodeDrawer {
 public:
  virtual void Draw(GLProgram program, Camera* camera) = 0;
  virtual ~NodeDrawer() = default;

 protected:
  std::vector<GLuint> vertex_size_;
  // Buffer id for vertex and color array
  std::vector<GLuint> vertex_buffer_;
  std::vector<GLuint> vertex_color_buffer_;
  Node* node_ = nullptr;
};

// Draw all the path related to a node
class NodePathDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePathDrawer(Node* node);
  // Use a precompiled program and a scale to draw
  void Draw(GLProgram program, Camera* camera);

 private:
  void GenPathBuffer(const Path&, bool);
  void GenBuffers();
};

// Draw all the polygons related to a node
class NodePolyDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePolyDrawer(Node* node);
  void Draw(GLProgram program, Camera* camera);

 private:
  void GenTriangleBuffer(const TriangleMesh&);
  void GenBuffers();

  std::vector<GLuint> index_buffer_;
  std::vector<GLuint> index_size_;
};

template <class DrawerType>
class Canvas {
 public:
  Canvas(GLProgram program, Camera* camera);
  void AddNode(Node* node);
  void RemoveNodeByID(id_t id);
  void Draw();

 private:  
  GLProgram program_;
  Camera* camera_;
  std::unordered_map<id_t, std::unique_ptr<DrawerType> > drawers_;
};

}  // namespace diagrammar

#endif  // RUNTIME_DRAW_DRAWER_H_