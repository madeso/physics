#pragma once

#include <vector>
#include <functional>
#include <optional>
#include <variant>


#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

/*
// struct with 3 floats for x, y, z or i + j + k
struct vector3
{
    float x, y, z = 0.0f;
};
*/

using quaternion = glm::quat;


using vector3 = glm::vec3;

// Object -> CollisionObject + Rigitbody
struct CollisionObject;
struct Rigidbody;



struct Collision;
struct Solver;

// PhysicsWorld -> CollisionWorld + DynamicsWorld
struct CollisionWorld;
struct DynamicsWorld;

struct CollisionPoints;
struct Transform;
