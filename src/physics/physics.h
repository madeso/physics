#pragma once

#include <vector>
#include <functional>
#include <optional>

#include "glm/vec3.hpp"

/*
// struct with 3 floats for x, y, z or i + j + k
struct vector3
{
    float x, y, z = 0.0f;
};
*/

struct quaternion
{
    float x, y, z, w = 0.0f;
};


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
struct Collider;
struct SphereCollider;
struct PlaneCollider;
