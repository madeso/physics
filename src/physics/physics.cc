#include "physics.h"
#include <type_traits>


namespace physics
{

CollisionPoints NoCollision()
{
	return {};
}


CollisionPoints TestCollision
(
	const Shape& lhs_collider, const Transform& lhs_transform,
	const Shape& rhs_collider, const Transform& rhs_transform
);
 
void SolveCollisions(const CollisionWorld& world, std::vector<Collision>& collisions, float dt)
{
	for (const Solver& solve : world.solvers)
	{
		solve(collisions, dt);
	}
}

void SendCollisionCallbacks(const CollisionWorld& world, std::vector<Collision>& collisions, float dt)
{
	auto call = [](const CollisionCallback& cb, Collision& collision, float dt)
	{
		if (cb) { cb(collision, dt); }
	};

	for (Collision& collision : collisions)
	{
		call(world.on_collision, collision, dt);
		call(collision.a->on_collision, collision, dt);
		call(collision.b->on_collision, collision, dt);
	}
}

void ResolveCollisions(CollisionWorld* world, float dt)
{
	std::vector<Collision> collisions;
	std::vector<Collision> triggers;

	// todo(Gustav): reduce the N*M complexity
	for (CollisionObject* a : world->objects)
	{
		for (CollisionObject* b : world->objects)
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

	SolveCollisions(*world, collisions, dt);
	// Don't solve triggers

	SendCollisionCallbacks(*world, collisions, dt);
	SendCollisionCallbacks(*world, triggers, dt);
}
 
void ApplyGravity(DynamicsWorld* world)
{
	for (CollisionObject* object : world->objects)
	{
		if (!object->body)
		{
			continue;
		}

		if (object->body->takes_gravity == false)
		{
			continue;
		}

		const auto gravity = object->body->custom_gravity.value_or(world->global_gravity);
		object->body->force += gravity * object->body->mass;
	}
}

void MoveObjects(DynamicsWorld* world, float dt)
{
	for (CollisionObject* object : world->objects)
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

void Step(DynamicsWorld* world, float dt)
{
	ApplyGravity(world);
	ResolveCollisions(world, dt);
	MoveObjects(world, dt);
}

namespace algo
{
	CollisionPoints FindSphereSphereCollisionPoints(const Sphere& a, const Transform& ta, const Sphere& b, const Transform& tb)
	{
		vector3 A = a.center + ta.position;
		vector3 B = b.center + tb.position;

		float Ar = a.radius * ta.scale;
		float Br = b.radius * tb.scale;

		vector3 AtoB = B - A;
		vector3 BtoA = A - B;

		if (glm::length(AtoB) > Ar + Br)
		{
			return NoCollision();
		}

		A += glm::normalize(AtoB) * Ar;
		B += glm::normalize(BtoA) * Br;

		AtoB = B - A;

		return
		{
			A, B,
			glm::normalize(AtoB),
			glm::length(AtoB),
			true
		};
	}

	CollisionPoints FindSpherePlaneCollisionPoints(const Sphere& a, const Transform& ta, const Plane& b, const Transform& tb)
	{
		vector3 A  = a.center + ta.position;
		float Ar = a.radius * ta.scale;

		vector3 N = b.normal * tb.rotation;
		N = glm::normalize(N);
		
		vector3 P = N * b.distance + tb.position;

		// distance from center of sphere to plane surface
		float d = glm::dot(A - P, N);

		if (d > Ar)
		{
			return NoCollision();
		}
		
		vector3 B = A - N * d;
		A = A - N * Ar;

		vector3 AtoB = B - A;

		return
		{
			A, B,
			glm::normalize(AtoB),
			glm::length(AtoB),
			true
		};
	}
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



void ImpulseSolver(std::vector<Collision>& collisions, float dt)
{
	for (Collision& coll : collisions)
	{
		// Replaces non dynamic objects with default values.

		auto& aBody = coll.a->body;
		auto& bBody = coll.b->body;

		glm::vec3 aVel = aBody ? aBody->velocity : glm::vec3(0.0f);
		glm::vec3 bVel = bBody ? bBody->velocity : glm::vec3(0.0f);
		glm::vec3 rVel = bVel - aVel;
		float  nSpd = glm::dot(rVel, coll.points.normal);

		float aInvMass = aBody ? (1.0f/aBody->mass): 1.0f;
		float bInvMass = bBody ? (1.0f/bBody->mass): 1.0f;

		// Impluse

		// This is important for convergence
		// a negitive impulse would drive the objects closer together
		if (nSpd >= 0)
		{
			continue;
		}

		float e = (aBody ? aBody->restitution : 1.0f) * (bBody ? bBody->restitution : 1.0f);

		float j = -(1.0f + e) * nSpd / (aInvMass + bInvMass);

		glm::vec3 impluse = j * coll.points.normal;

		if (aBody ? aBody->is_simulated : false)
		{
			aVel -= impluse * aInvMass;
		}

		if (bBody ? bBody->is_simulated : false)
		{
			bVel += impluse * bInvMass;
		}

		// Friction
		rVel = bVel - aVel;
		nSpd = glm::dot(rVel, coll.points.normal);

		glm::vec3 tangent = rVel - nSpd * coll.points.normal;

		// safe normalize
		if (glm::length2(tangent) > 0.0001f)
		{
			tangent = glm::normalize(tangent);
		}

		const float fVel = glm::dot(rVel, tangent);
		const float aSF = aBody ? aBody->static_friction : 0.0f;
		const float bSF = bBody ? bBody->static_friction : 0.0f;
		const float aDF = aBody ? aBody->dynamic_friction : 0.0f;
		const float bDF = bBody ? bBody->dynamic_friction : 0.0f;
		const float mu = (float)glm::vec2(aSF, bSF).length();
		const float f = -fVel / (aInvMass + bInvMass);

		glm::vec3 friction = [&]()
		{
			if (std::abs(f) < j * mu)
			{
				return f * tangent;
			}
			else
			{
				const auto new_mu = glm::length(glm::vec2(aDF, bDF));
				return -j * tangent * new_mu;
			}
		}();

		if (aBody ? aBody->is_simulated : false)
		{
			aBody->velocity = aVel - friction * aInvMass;
		}

		if (bBody ? bBody->is_simulated : false)
		{
			bBody->velocity = bVel + friction * bInvMass;
		}
	}
}


void PositionSolver(std::vector<Collision>& collisions, float dt)
{
	for (Collision& coll : collisions)
	{
		CollisionObject* aBody = coll.a;
		CollisionObject* bBody = coll.b;

		float aStatic = aBody->body.has_value() ? 1.0f : 0.0f;
		float bStatic = bBody->body.has_value() ? 1.0f : 0.0f;

		auto resolution = coll.points.normal * coll.points.depth / std::max(1.0f, aStatic + bStatic);

		aBody->transform.position -= resolution * (1.0f - aStatic);
		bBody->transform.position += resolution * (1.0f - bStatic);
	}
}


}

