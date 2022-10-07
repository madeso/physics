#include "physics.h"



struct Transform
{
	vector3 position;
	vector3 scale;
	quaternion rotation;
};


struct Sphere
{
	vector3 Center;
	float Radius;
};

struct Plane
{
	vector3 Plane;
	float Distance;
};

using CollisionCallback = std::function<void(Collision&, float)>;

struct Rigidbody
{
	std::optional<vector3> custom_gravity;
	vector3 force;    // Net force
	vector3 velocity;

	float mass = 10.0f;
	bool takes_gravity = true; // If the rigidbody will take gravity from the world.

	float static_friction = 0.5f;  // Static friction coefficient
	float dynamic_friction = 0.5f; // Dynamic friction coefficient
	float restitution = 0.1f;     // Elasticity of collisions (bounciness)
};

struct CollisionObject
{
	Transform transform;
	Collider* collider;
	bool is_trigger;
	std::optional<Rigidbody> body;
 
	CollisionCallback on_collision;
};


struct CollisionPoints
{
	vector3 a; // Furthest point of a into b
	vector3 b; // Furthest point of b into a
	vector3 normal; // b – a normalized
	float depth = 0;    // Length of b – a
	bool has_collision = false;
};

struct Collision
{
    CollisionObject* a;
    CollisionObject* b;
    CollisionPoints points;
};

struct Solver
{
	virtual ~Solver() = default;
	virtual void Solve(std::vector<Collision>& collisions, float dt) = 0;
};

struct Collider
{
	virtual ~Collider() = default;
	virtual CollisionPoints TestCollision(const Transform& transform, const Collider* collider, const Transform& colliderTransform) const = 0;
	virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider* sphere, const Transform& sphereTransform) const = 0;
	virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider* plane, const Transform& planeTransform) const = 0;
};

struct CollisionWorld
{
	std::vector<CollisionObject*> objects;
	std::vector<Solver*> solvers;
	CollisionCallback on_collision;
 
	void SolveCollisions(std::vector<Collision>& collisions, float dt)
	{
		for (Solver* solver : solvers)
		{
			solver->Solve(collisions, dt);
		}
	}
 
	void SendCollisionCallbacks(std::vector<Collision>& collisions, float dt) const
	{
		auto call = [](const CollisionCallback& cb, Collision& collision, float dt)
		{
			if (cb) { cb(collision, dt); }
		};

		for (Collision& collision : collisions)
		{
			call(on_collision, collision, dt);
			call(collision.a->on_collision, collision, dt);
			call(collision.b->on_collision, collision, dt);
		}
	}
 
	void ResolveCollisions(float dt)
	{
		std::vector<Collision> collisions;
		std::vector<Collision> triggers;

		// todo(Gustav): reduce the N*M complexity
		for (CollisionObject* a : objects)
		{
			for (CollisionObject* b : objects)
			{
				if (a == b)
				{
					// todo(Gustav): is this properly iteration over all objects?
					break;
				}

				const auto both_has_collision = a->collider || b->collider;
				if(both_has_collision == false)
				{
					continue;
				}
 
				CollisionPoints points = a->collider->TestCollision(a->transform, b->collider, b->transform);
 
				if (points.has_collision)
				{
					const auto any_is_trigger = a->is_trigger || b->is_trigger;
					if (any_is_trigger)
					{
						triggers.emplace_back(Collision{a, b, points});
					}
					else
					{
						collisions.emplace_back(Collision{a, b, points});
					}
				}
			}
		}
 
		SolveCollisions(collisions, dt);
		// Don't solve triggers
 
		SendCollisionCallbacks(collisions, dt);
		SendCollisionCallbacks(triggers, dt);
	}
};

struct DynamicsWorld : public CollisionWorld
{
	vector3 global_gravity = vector3{ 0, -9.81f, 0 };
 
	void ApplyGravity()
	{
		for (CollisionObject* object : objects)
		{
			if (!object->body)
			{
				continue;
			}

			if (object->body->takes_gravity == false)
			{
				continue;
			}

			const auto gravity = object->body->custom_gravity.value_or(global_gravity);
			object->body->force += gravity * object->body->mass;
		}
	}
 
	void MoveObjects(float dt)
	{
		for (CollisionObject* object : objects)
		{
			if (!object->body)
			{
				continue;
			}
 
			object->body->velocity += object->body->force / object->body->mass * dt;
			object->transform.position += object->body->velocity * dt;
			object->body->force = {0, 0, 0};
		}
	}
 
	void Step(float dt)
	{
		ApplyGravity();
		ResolveCollisions(dt);
		MoveObjects(dt);
	}
};

namespace algo
{
	CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider* a, const Transform& ta, const SphereCollider* b, const Transform& tb);
	CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider* a, const Transform& ta, const PlaneCollider* b, const Transform& tb);
}

struct SphereCollider : Collider
{
	vector3 Center;
	float Radius;
 
	CollisionPoints TestCollision(const Transform& transform, const Collider* collider, const Transform& colliderTransform) const override
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}
 
	CollisionPoints TestCollision(const Transform& transform, const SphereCollider* sphere, const Transform& sphereTransform) const override
	{
		return algo::FindSphereSphereCollisionPoints(this, transform, sphere, sphereTransform);
	}
 
	CollisionPoints TestCollision(const Transform& transform, const PlaneCollider* plane, const Transform& planeTransform) const override
	{
		return algo::FindSpherePlaneCollisionPoints(this, transform, plane, planeTransform);
	}
};

struct PlaneCollider : Collider
{
	vector3 Plane;
	float Distance;
 
	CollisionPoints TestCollision(const Transform& transform, const Collider* collider, const Transform& colliderTransform) const override
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}
 
	CollisionPoints TestCollision(const Transform& transform, const SphereCollider* sphere, const Transform& sphereTransform) const override
	{
		// reuse sphere code
		CollisionPoints points = sphere->TestCollision(sphereTransform, this, transform);

		vector3 T = points.a; // You could have an algo Plane v Sphere to do the swap
		points.a = points.b;
		points.b = T;

		points.normal = -points.normal;

		return points;
	}
 
	CollisionPoints TestCollision(const Transform& transform, const PlaneCollider* plane, const Transform& planeTransform) const override
	{
		return {}; // No plane v plane
	}
};
