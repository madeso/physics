#pragma once

#include <vector>
#include <functional>
#include <optional>
#include <variant>


#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"


namespace physics
{


using quaternion = glm::quat;
using vector3 = glm::vec3;


struct Transform
{
	vector3 position;
	float scale;
	quaternion rotation;
};


struct Sphere
{
	vector3 center;
	float radius;
};

struct Plane
{
	vector3 normal;
	float distance;
};

struct NullShape {};


struct Rigidbody
{
	bool is_simulated = true;
	std::optional<vector3> custom_gravity;
	vector3 force;    // Net force
	vector3 velocity;

	float mass = 10.0f;
	bool takes_gravity = true; // If the rigidbody will take gravity from the world.

	float static_friction = 0.5f;  // Static friction coefficient
	float dynamic_friction = 0.5f; // Dynamic friction coefficient
	float restitution = 0.1f;     // Elasticity of collisions (bounciness)
};

using Shape = std::variant<Sphere, Plane, NullShape>;


struct CollisionBetween;
using OnCollision = std::function<void(CollisionBetween&, float)>;

struct Object
{
	Transform transform;
	Shape shape = NullShape{};
	bool is_trigger;
	std::optional<Rigidbody> body; 
	OnCollision on_collision;
};

struct Collision
{
	vector3 a; // Furthest point of a into b
	vector3 b; // Furthest point of b into a
	vector3 normal; // b – a normalized
	float depth = 0;    // Length of b – a
	bool has_collision = false;
};

Collision CreateCollision(const vector3& a, const vector3& b);
Collision NoCollision();

struct CollisionBetween
{
    Object* a;
    Object* b;
    Collision collision;
};

using Solver = std::function<void (std::vector<CollisionBetween>&, float)>;
void ImpulseSolver(std::vector<CollisionBetween>& collisions, float dt);
void PositionSolver(std::vector<CollisionBetween>& collisions, float dt);


struct World
{
	std::vector<Object*> objects;
	std::vector<Solver> solvers;
	OnCollision on_collision;

    bool apply_gravity = true;
    vector3 global_gravity = vector3{ 0, -9.81f, 0 };
};



void Step(World* world, float dt);


}
