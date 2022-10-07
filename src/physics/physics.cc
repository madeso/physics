#include "physics.h"



struct Transform
{
	vector3 Position;
	vector3 Scale;
	quaternion Rotation;
};

struct CollisionObject
{
	Transform transform;
	Collider* collider;
	bool is_trigger;
 
	std::function<void(Collision&, float)> on_collision;

	// return true if this is rigidbody
	bool IsDynamic() const;
};


struct Rigidbody : CollisionObject
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


struct CollisionPoints
{
	vector3 A; // Furthest point of A into B
	vector3 B; // Furthest point of B into A
	vector3 Normal; // B – A normalized
	float Depth = 0;    // Length of B – A
	bool HasCollision = false;
};

struct Collision
{
    CollisionObject* ObjA;
    CollisionObject* ObjB;
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
protected:
	std::vector<CollisionObject*> m_objects;
	std::vector<Solver*> m_solvers;
 
	std::function<void(Collision&, float)> m_onCollision;
 
public:
	void AddCollisionObject   (CollisionObject* object);
	void RemoveCollisionObject(CollisionObject* object);
	void AddSolver   (Solver* solver);
	void RemoveSolver(Solver* solver);
	void SetCollisionCallback(std::function<void(Collision&, float)>& callback);
 
	void SolveCollisions(std::vector<Collision>& collisions, float dt)
	{
		for (Solver* solver : m_solvers)
		{
			solver->Solve(collisions, dt);
		}
	}
 
	void SendCollisionCallbacks(std::vector<Collision>& collisions, float dt)
	{
		for (Collision& collision : collisions)
		{
			m_onCollision(collision, dt);
 
			auto& a = collision.ObjA->on_collision;
			auto& b = collision.ObjB->on_collision;
 
			if (a) a(collision, dt);
			if (b) b(collision, dt);
		}
	}
 
	void ResolveCollisions(float dt)
	{
		std::vector<Collision> collisions;
		std::vector<Collision> triggers;

		// todo(Gustav): reduce the N*M complexity
		for (CollisionObject* a : m_objects)
		{
			for (CollisionObject* b : m_objects)
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
 
				if (points.HasCollision)
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
 
		SolveCollisions(collisions, dt); // Don't solve triggers
 
		SendCollisionCallbacks(collisions, dt);
		SendCollisionCallbacks(triggers, dt);
	}
};

struct DynamicsWorld : public CollisionWorld
{
private:
	vector3 global_gravity = vector3{ 0, -9.81f, 0 };
 
public:
	void AddRigidbody(Rigidbody* rigidbody)
	{
		AddCollisionObject(rigidbody);
	}
 
	void ApplyGravity()
	{
		for (CollisionObject* object : m_objects)
		{
			if (object->IsDynamic() == false)
			{
				continue;
			}

			Rigidbody* rigidbody = (Rigidbody*)object;

			if (rigidbody->takes_gravity == false)
			{
				continue;
			}

			const auto gravity = rigidbody->custom_gravity.value_or(global_gravity);
			rigidbody->force += gravity * rigidbody->mass;
		}
	}
 
	void MoveObjects(float dt)
	{
		for (CollisionObject* object : m_objects)
		{
			if (!object->IsDynamic())
			{
				continue;
			}
 
			Rigidbody* rigidbody = (Rigidbody*)object;
 
			rigidbody->velocity = rigidbody->velocity + rigidbody->force / rigidbody->mass * dt;
			rigidbody->transform.Position = rigidbody->transform.Position + rigidbody->velocity * dt;
			rigidbody->force = {0, 0, 0};
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

struct PlaneCollider
	: Collider
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

		vector3 T = points.A; // You could have an algo Plane v Sphere to do the swap
		points.A = points.B;
		points.B = T;

		points.Normal = -points.Normal;

		return points;
	}
 
	CollisionPoints TestCollision(const Transform& transform, const PlaneCollider* plane, const Transform& planeTransform) const override
	{
		return {}; // No plane v plane
	}
};
