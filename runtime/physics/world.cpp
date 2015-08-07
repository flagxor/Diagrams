// Copyright 2015 Native Client Authors.

#include <json/json.h>

#include <random>
#include <iostream>
#include <algorithm>

#include "physics/world.h"
#include "utility/world_parser.h"
#include "physics/physics_engine_liquidfun.h"
#include "physics/physics_engine_bullet.h"

namespace {
// Demo state variables goes here
bool left_flipper_pressed = false;
bool right_flipper_pressed = false;
}  // namespace

namespace diagrammar {

World::World() {}

World::~World() {}

void World::Start(EngineType engine_type) {
  auto engine_ptr = physics_engine_.release();
  if (engine_ptr) delete engine_ptr;

  // Now generate some sample particles
  if (true) {
    std::random_device rd_device;
    std::default_random_engine rd_generator(rd_device());
    std::uniform_real_distribution<float> pos_distx(-300, 0.f);
    std::uniform_real_distribution<float> pos_disty(100.f, 150.f);
    std::uniform_real_distribution<float> vel_dist(-250.f, 250.f);

    // create a disk
    for (int i = 0; i < 10; ++i) {
      auto node_ptr = make_unique<Node>();
      node_ptr->collision_shapes.emplace_back(make_unique<Disk2D>(10.0));
      node_ptr->motion_type = MotionType::kDynamic;
      node_ptr->material_info.restitution = 0.8;
      node_ptr->frame.SetTranslation(
          Vector3f(pos_distx(rd_generator), pos_disty(rd_generator), 0));
      node_ptr->velocity =
          Vector3f(vel_dist(rd_generator), vel_dist(rd_generator), 0);
      AddNodeInternal(std::move(node_ptr));
    }
  }

  // TODO(tingnan) add more physics engines
  switch (engine_type) {
    case kLiquidFun:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
    case kBullet:
      physics_engine_.reset(new PhysicsEngineBullet(time_step()));
      break;
    default:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
  }

  for (size_t index = 0; index < node_map_.size(); ++index) {
    physics_engine_->AddNode(node_map_.get(index).get());
  }

  for (size_t index = 0; index < joint_map_.size(); ++index) {
    physics_engine_->AddJoint(joint_map_.get(index).get());
  }

  timer_.Initialize();
}

void World::Step() {
  if (!physics_engine_) return;

  // The timer returns the proper number of steps the engine needs to execute
  // If the elapsed time is too long, only one step is returned.
  int num_ticks = timer_.BeginNextFrame();
  assert(num_ticks >= 0);

  double before_step = timer_.now();
  for (int tick = num_ticks; tick > 0; --tick) {
    physics_engine_->Step();
  }
  double after_step = timer_.now();

  if (num_ticks > 0) {
    step_time_.push_back((after_step - before_step) / num_ticks);
    if (step_time_.size() > 200) {
      step_time_.pop_front();
    }
  }

  if (timer_.accumulated_ticks() % 200 == 0 && timer_.accumulated_ticks() > 0) {
    std::cout << "200 counts: ";
    auto result = std::minmax_element(step_time_.begin(), step_time_.end());
    std::cout << "min: " << *result.first << " ms ";
    std::cout << "max: " << *result.second << " ms ";
    std::cout << "mean: "
              << std::accumulate(step_time_.begin(), step_time_.end(), 0.0) /
                     step_time_.size()
              << " ms\n";
  }

  physics_engine_->SendDataToWorld();
}

bool World::HandleMessage(const Json::Value& message) {
  // All the code in the following brackets shows how the world can handle
  // message and manipulate nodes in the underlying physics engine.
  {
    const id_t left_flipper_id = 11;
    const id_t right_flipper_id = 12;
    //
    const Vector3f torque(0, 0, 15000);
    const Vector3f angular_impulse(0, 0, 9000);

    std::random_device rd_device;
    std::uniform_real_distribution<float> impulse_dist(0.f, 250.f);
    std::default_random_engine rd_generator(rd_device());

    if (left_flipper_pressed) {
      // Apply a torque
      physics_engine_->ApplyTorqueToNode(left_flipper_id, torque);
    }

    if (right_flipper_pressed) {
      // Apply a torque
      physics_engine_->ApplyTorqueToNode(right_flipper_id, -torque);
    }

    // Handed coded play rule for now
    if (message["type"] == "key") {
      if (message["key_code"] == "A" && message["key_pressed"] == true) {
        if (!left_flipper_pressed) {
          // Apply an impulse
          physics_engine_->ApplyAngularImpulseToNode(left_flipper_id,
                                                     angular_impulse);
        }
        left_flipper_pressed = true;
      }

      if (message["key_code"] == "A" && message["key_pressed"] == false) {
        physics_engine_->ApplyAngularImpulseToNode(left_flipper_id,
                                                   -1.25 * angular_impulse);
        left_flipper_pressed = false;
      }

      if (message["key_code"] == "D" && message["key_pressed"] == true) {
        if (!right_flipper_pressed) {
          // Apply an angular impulse initially
          physics_engine_->ApplyAngularImpulseToNode(right_flipper_id,
                                                     -angular_impulse);
        }
        right_flipper_pressed = true;
      }

      if (message["key_code"] == "D" && message["key_pressed"] == false) {
        physics_engine_->ApplyAngularImpulseToNode(right_flipper_id,
                                                   1.25 * angular_impulse);
        right_flipper_pressed = false;
      }

      if (message["key_code"] == "B" && message["key_pressed"] == true) {
        // boost all particles
        for (decltype(node_map_)::const_iterator itr = node_map_.cbegin();
             itr != node_map_.cend(); ++itr) {
          if (itr->first >= 14) {
            Vector3f impulse(impulse_dist(rd_generator),
                             impulse_dist(rd_generator), 0);
            physics_engine_->ApplyImpulseToNode(itr->first, impulse,
                                                Vector3f(0, 0, 0));
          }
        }
      }

      if (message["key_code"] == "R" && message["key_pressed"] == true) {
      }
    }
  }
  return true;
}

const Node* World::AddNode(std::unique_ptr<Node> new_node) {
  id_t ext_id = new_node->id;
  // Create the id mapping
  Node* node_ptr = AddNodeInternal(std::move(new_node));
  node_id_map_.insert(IDMap::value_type(ext_id, node_ptr->id));
  return node_ptr;
}

Node* World::AddNodeInternal(std::unique_ptr<Node> new_node) {
  id_t int_id = id_pool_.GetID();
  // Move the content to the map
  node_map_[int_id] = std::move(new_node);
  Node* node_ptr = node_map_[int_id].get();
  node_ptr->id = int_id;
  // Now we also would like to add the same node from the underlying
  // engine
  if (physics_engine_) {
    physics_engine_->AddNode(node_ptr);
  }

  return node_ptr;
}
// TODO(tingan) When remove a node, we also need to remove all the joints
// connected to it
std::unique_ptr<Node> World::RemoveNodeByIntID(id_t id) {
  std::unique_ptr<Node> node_ptr;
  auto itr = node_map_.find(id);
  if (itr != node_map_.end()) {
    node_ptr = std::move(itr->second);
  }
  node_map_.erase(id);
  node_id_map_.by<internal_id>().erase(id);
  id_pool_.RecycleID(id);

  {
    // We may want to also delete joints that use the node,
    // or simply left for the user to handle the orphaned joint.
    for (auto itr = joint_map_.begin(); itr != joint_map_.end(); ++itr) {
      Joint* joint_ptr = itr->second.get();
      if (joint_ptr->node_1 == id || joint_ptr->node_2 == id) {
        // Also destroy the joint.
        RemoveJointByIntID(joint_ptr->id);
      }
    }
  }

  if (physics_engine_) {
    physics_engine_->RemoveNodeByID(id);
  }
  /*
  std::cout << "int id: " << id << std::endl;
  std::cout << "int id table looks like: ";
  for (auto itr = node_id_map_.left.begin(); itr != node_id_map_.left.end();
       ++itr) {
    std::cout << itr->first << " " << itr->second << std::endl;
  }*/
  return node_ptr;
}

std::unique_ptr<Node> World::RemoveNodeByExtID(id_t id) {
  auto itr = node_id_map_.by<external_id>().find(id);
  if (itr != node_id_map_.by<external_id>().end()) {
    return RemoveNodeByIntID(itr->get<internal_id>());
  }
  return std::unique_ptr<Node>(nullptr);
}

const Node* World::GetNodeByIntID(id_t id) const {
  auto const_itr = node_map_.find(id);
  if (const_itr != node_map_.end()) {
    return const_itr->second.get();
  }
  return nullptr;
}

const Node* World::GetNodeByExtID(id_t id) const {
  auto itr = node_id_map_.by<external_id>().find(id);
  if (itr != node_id_map_.by<external_id>().end()) {
    assert(node_map_.find(itr->get<internal_id>()) != node_map_.end());
    return GetNodeByIntID(itr->get<internal_id>());
  }
  return nullptr;
}

const Node* World::GetNodeByIndex(size_t index) const {
  assert(index < node_map_.size());
  return node_map_.get(index).get();
}

size_t World::GetNumNodes() const { return node_map_.size(); }

const Joint* World::AddJoint(std::unique_ptr<Joint> new_joint) {
  id_t ext_id = new_joint->id;
  // Find the nodes internal used id;
  id_t ext_node_1 = new_joint->node_1;
  id_t ext_node_2 = new_joint->node_2;
  new_joint->node_1 = node_id_map_.by<external_id>().at(ext_node_1);
  new_joint->node_2 = node_id_map_.by<external_id>().at(ext_node_2);
  Joint* joint_ptr = AddJointInternal(std::move(new_joint));
  joint_id_map_.insert(IDMap::value_type(ext_id, joint_ptr->id));
  // std::cerr << ext_id << " " << joint_ptr->id << std::endl;
  // std::cerr << ext_node_1 << " " << joint_ptr->node_1 << std::endl;
  // std::cerr << ext_node_2 << " " << joint_ptr->node_2 << std::endl;
  return joint_ptr;
}

Joint* World::AddJointInternal(std::unique_ptr<Joint> new_joint) {
  // Move the content to the map
  id_t joint_id = id_pool_.GetID();
  joint_map_[joint_id] = std::move(new_joint);
  Joint* joint_ptr = joint_map_[joint_id].get();
  joint_ptr->id = joint_id;
  // Now we also would like to add the same node from the underlying
  // engine
  if (physics_engine_) {
    physics_engine_->AddJoint(joint_ptr);
  }
  return joint_ptr;
}

std::unique_ptr<Joint> World::RemoveJointByExtID(id_t id) {
  auto itr = joint_id_map_.by<external_id>().find(id);
  if (itr != joint_id_map_.by<external_id>().end()) {
    return RemoveJointByIntID(itr->get<internal_id>());
  }
  return std::unique_ptr<Joint>(nullptr);
}

std::unique_ptr<Joint> World::RemoveJointByIntID(id_t id) {
  std::unique_ptr<Joint> joint_ptr;
  auto itr = joint_map_.find(id);
  if (itr != joint_map_.end()) {
    joint_ptr = std::move(itr->second);
  }
  joint_map_.erase(id);
  joint_id_map_.by<internal_id>().erase(id);
  id_pool_.RecycleID(id);
  if (physics_engine_) {
    physics_engine_->RemoveJointByID(id);
  }
  return joint_ptr;
}

const Joint* World::GetJointByIntID(id_t id) const {
  auto const_itr = joint_map_.find(id);
  if (const_itr != joint_map_.end()) {
    return const_itr->second.get();
  }
  return nullptr;
}

const Joint* World::GetJointByExtID(id_t id) const {
  auto itr = joint_id_map_.by<external_id>().find(id);
  if (itr != joint_id_map_.by<external_id>().end()) {
    assert(joint_map_.find(itr->get<internal_id>()) != joint_map_.end());
    return GetJointByIntID(itr->get<internal_id>());
  }
  return nullptr;
}

float World::time_step() const { return timer_.tick_time(); }

float World::now() const { return timer_.now(); }

float World::simulation_time() const {
  return timer_.accumulated_ticks() * timer_.tick_time();
}

}  // namespace diagrammar
