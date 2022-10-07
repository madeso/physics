#include "physics.h"
#include <type_traits>


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

struct NullShape {};

using CollisionCallback = std::function<void(Collision&, float)>;

using Shape = std::variant<Sphere, Plane, NullShape>;

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
	Shape shape = NullShape{};
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

CollisionPoints NoCollision()
{
	return {};
}

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

CollisionPoints TestCollision
(
	const Shape& lhs_collider, const Transform& lhs_transform,
	const Shape& rhs_collider, const Transform& rhs_transform
);

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
 
				CollisionPoints points = TestCollision(a->shape, a->transform, b->shape, b->transform);
 
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
	CollisionPoints FindSphereSphereCollisionPoints(const Sphere& a, const Transform& ta, const Sphere& b, const Transform& tb);
	CollisionPoints FindSpherePlaneCollisionPoints(const Sphere& a, const Transform& ta, const Plane& b, const Transform& tb);
}

namespace forwarders
{
	CollisionPoints TestCollision(const Sphere& self, const Transform& transform, const Sphere& sphere, const Transform& sphereTransform)
	{
		return algo::FindSphereSphereCollisionPoints(self, transform, sphere, sphereTransform);
	}

	CollisionPoints TestCollision(const Sphere& self, const Transform& transform, const Plane& plane, const Transform& planeTransform)
	{
		return algo::FindSpherePlaneCollisionPoints(self, transform, plane, planeTransform);
	}

	CollisionPoints TestCollision(const Plane& self, const Transform& transform, const Sphere& sphere, const Transform& sphereTransform)
	{
		// reuse sphere code
		CollisionPoints points = TestCollision(sphere, sphereTransform, self, transform);

		vector3 T = points.a; // You could have an algo Plane v Sphere to do the swap
		points.a = points.b;
		points.b = T;

		points.normal = -points.normal;

		return points;
	}

	CollisionPoints TestCollision(const Plane& self, const Transform& transform, const Plane& plane, const Transform& planeTransform)
	{
		return NoCollision();
	}
}

template<class> inline constexpr bool always_false_v = false;

namespace intermediate
{
	CollisionPoints TestCollision
	(
		const Sphere& lhs_collider, const Transform& lhs_transform,
		const Shape& rhs_collider, const Transform& rhs_transform
	)
	{
		return std::visit([&](const auto& rhs) -> CollisionPoints
			{
				using T = std::decay_t<decltype(rhs)>;
				if constexpr (std::is_same_v<T, Sphere>)
				{
					return forwarders::TestCollision(lhs_collider, lhs_transform, rhs, rhs_transform);
				}
				else if constexpr (std::is_same_v<T, Plane>)
				{
					return forwarders::TestCollision(lhs_collider, lhs_transform, rhs, rhs_transform);
				}
				else if constexpr (std::is_same_v<T, NullShape>)
				{
					return NoCollision();
				}
				else
				{
					static_assert(always_false_v<T>, "non-exhaustive visitor!");
				}
			}, rhs_collider);
	}

	CollisionPoints TestCollision
	(
		const Plane& lhs_collider, const Transform& lhs_transform,
		const Shape& rhs_collider, const Transform& rhs_transform
	)
	{
		return std::visit([&](const auto& rhs) -> CollisionPoints
			{
				using T = std::decay_t<decltype(rhs)>;
				if constexpr (std::is_same_v<T, Sphere>)
				{
					return forwarders::TestCollision(lhs_collider, lhs_transform, rhs, rhs_transform);
				}
				else if constexpr (std::is_same_v<T, Plane>)
				{
					return forwarders::TestCollision(lhs_collider, lhs_transform, rhs, rhs_transform);
				}
				else if constexpr (std::is_same_v<T, NullShape>)
				{
					return NoCollision();
				}
				else
				{
					static_assert(always_false_v<T>, "non-exhaustive visitor!");
				}
			}, rhs_collider);
	}
}

CollisionPoints TestCollision
(
	const Shape& lhs_collider, const Transform& lhs_transform,
	const Shape& rhs_collider, const Transform& rhs_transform
)
{
	return std::visit([&](const auto& lhs) -> CollisionPoints
	{
		using T = std::decay_t<decltype(lhs)>;
		if constexpr (std::is_same_v<T, Sphere>)
		{
			return intermediate::TestCollision(lhs, lhs_transform, rhs_collider, rhs_transform);
		}
		else if constexpr (std::is_same_v<T, Plane>)
		{
			return intermediate::TestCollision(lhs, lhs_transform, rhs_collider, rhs_transform);
		}
		else if constexpr (std::is_same_v<T, NullShape>)
		{
			return NoCollision();
		}
		else
		{
			static_assert(always_false_v<T>, "non-exhaustive visitor!");
		}
	}, lhs_collider);
}
