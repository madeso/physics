#include "physics.h"



struct Transform
{
	vector3 Position;
	vector3 Scale;
	quaternion Rotation;
};

struct CollisionObject
{
protected:
	Transform m_transform;
	Collider* m_collider;
	bool m_isTrigger;
	bool m_isDynamic;
 
	std::function<void(Collision&, float)> m_onCollision;
 
public:
	// getters & setters, no setter for isDynamic

	auto& OnCollision()
	{
		return m_onCollision;
	}

	auto Col()
	{
		return m_collider;
	}

	const Transform* Trans()
	{
		return &m_transform;
	}

	bool IsTrigger() const
	{
		return m_isTrigger;
	}

	bool IsDynamic() const
	{
		return m_isDynamic;
	}

	void SetPosition(const vector3 g)
	{
		m_transform.Position = g;
	}

	const vector3 Position() const
	{
		return m_transform.Position;
	}
};


struct Rigidbody : CollisionObject
{
private:
	vector3 m_gravity;  // Gravitational acceleration
	vector3 m_force;    // Net force
	vector3 m_velocity;
 
	float m_mass;
	bool m_takesGravity; // If the rigidbody will take gravity from the world.
 
	float m_staticFriction;  // Static friction coefficient
	float m_dynamicFriction; // Dynamic friction coefficient
	float m_restitution;     // Elasticity of collisions (bounciness)
 
public:
	bool TakesGravity() const
	{
		return m_takesGravity;
	}

	float Mass() const
	{
		return m_mass;
	}

	void SetGravity(const vector3 g)
	{
		m_gravity = g;
	}

	const vector3& Gravity() const
	{
		return m_gravity;
	}

	void SetVelocity(const vector3 g)
	{
		m_velocity = g;
	}

	const vector3& Velocity() const
	{
		return m_velocity;
	}

	void ApplyForce(const vector3& v)
	{
		m_force += v;
	}

	void SetForce(const vector3 g)
	{
		m_force = g;
	}

	const vector3& Force() const
	{
		return m_force;
	}
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
	virtual CollisionPoints TestCollision(const Transform* transform, const Collider* collider, const Transform* colliderTransform) const = 0;
	virtual CollisionPoints TestCollision(const Transform* transform, const SphereCollider* sphere, const Transform* sphereTransform) const = 0;
	virtual CollisionPoints TestCollision(const Transform* transform, const PlaneCollider* plane, const Transform* planeTransform) const = 0;
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
 
			auto& a = collision.ObjA->OnCollision();
			auto& b = collision.ObjB->OnCollision();
 
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

				const auto both_has_collision = a->Col() || b->Col();
				if(both_has_collision == false)
				{
					continue;
				}
 
				CollisionPoints points = a->Col()->TestCollision( a->Trans(), b->Col(), b->Trans());
 
				if (points.HasCollision)
				{
					const auto any_is_trigger = a->IsTrigger() || b->IsTrigger();
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
	vector3 m_gravity = vector3{ 0, -9.81f, 0 };
 
public:
	void AddRigidbody(Rigidbody* rigidbody)
	{
		if (rigidbody->TakesGravity())
		{
			rigidbody->SetGravity(m_gravity);
		}
 
		AddCollisionObject(rigidbody);
	}
 
	void ApplyGravity()
	{
		for (CollisionObject* object : m_objects)
		{
			if (!object->IsDynamic())
			{
				continue;
			}
 
			Rigidbody* rigidbody = (Rigidbody*)object;
			rigidbody->ApplyForce(rigidbody->Gravity() * rigidbody->Mass());
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
 
			vector3 vel = rigidbody->Velocity() + rigidbody->Force() / rigidbody->Mass() * dt;
 
			rigidbody->SetVelocity(vel);

			vector3 pos = rigidbody->Position() + rigidbody->Velocity() * dt;
 
			rigidbody->SetPosition(pos);
 
			rigidbody->SetForce(vector3{0, 0, 0});
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
	CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider* a, const Transform* ta, const SphereCollider* b, const Transform* tb);
	CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider* a, const Transform* ta, const PlaneCollider* b, const Transform* tb);
}

struct SphereCollider : Collider
{
	vector3 Center;
	float Radius;
 
	CollisionPoints TestCollision( const Transform* transform, const Collider* collider, const Transform* colliderTransform) const override
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}
 
	CollisionPoints TestCollision( const Transform* transform, const SphereCollider* sphere, const Transform* sphereTransform) const override
	{
		return algo::FindSphereSphereCollisionPoints( this, transform, sphere, sphereTransform);
	}
 
	CollisionPoints TestCollision( const Transform* transform, const PlaneCollider* plane, const Transform* planeTransform) const override
	{
		return algo::FindSpherePlaneCollisionPoints( this, transform, plane, planeTransform);
	}
};

struct PlaneCollider
	: Collider
{
	vector3 Plane;
	float Distance;
 
	CollisionPoints TestCollision( const Transform* transform, const Collider* collider, const Transform* colliderTransform) const override
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}
 
	CollisionPoints TestCollision( const Transform* transform, const SphereCollider* sphere, const Transform* sphereTransform) const override;
 
	CollisionPoints TestCollision( const Transform* transform, const PlaneCollider* plane, const Transform* planeTransform) const override
	{
		return {}; // No plane v plane
	}
};

CollisionPoints PlaneCollider::TestCollision(const Transform* transform, const SphereCollider* sphere, const Transform* sphereTransform) const
{
	// reuse sphere code
	CollisionPoints points = sphere->TestCollision(sphereTransform, this, transform);
 
	vector3 T = points.A; // You could have an algo Plane v Sphere to do the swap
	points.A = points.B;
	points.B = T;
 
	points.Normal = -points.Normal;
 
	return points;
}

