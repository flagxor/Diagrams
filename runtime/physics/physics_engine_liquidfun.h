// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_

#include <vector>
#include <unordered_map>

#include "physics/physics_engine.h"

class b2World;
class b2Body;
class b2Joint;

namespace diagrammar {
struct Node;
class JointDestructionListener;
class PhysicsEngineLiquidFun : public PhysicsEngine {
 public:
  // This engine is special in that it will directly modify
  // the objects in the world
  // other physics engines only obtain immutable copies
  explicit PhysicsEngineLiquidFun(float time_step);
  ~PhysicsEngineLiquidFun();

  void SendDataToWorld();
  void Step();
  void AddNode(Node* node);
  // The joint CANNOT be initialized, if the nodes it connect are not there yet
  void AddJoint(Joint* joint);
  void RemoveNodeByID(id_t id);
  void RemoveJointByID(id_t id);

  void ApplyForceToNode(id_t, const Vector2f& force, const Vector2f& offset);
  void ApplyImpulseToNode(id_t, const Vector2f& impulse,
                          const Vector2f& offset);
  void ApplyTorqueToNode(id_t, float torque);
  void ApplyAngularImpulseToNode(id_t, float torque);

 private:
  std::unique_ptr<b2World> b2world_;
  std::unordered_map<id_t, b2Body*> body_table_;
  std::unordered_map<id_t, b2Joint*> joint_table_;
  std::unique_ptr<JointDestructionListener> joint_destruction_listener_;

  // Used internally for the LCP solver
  // DONOT modify unless you know what it is
  int velocity_iterations_ = 5;
  int position_iterations_ = 5;

  // The engine is running on a different scale
  const float kScaleUp = 40;
  const float kScaleDown = 0.025f;
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
