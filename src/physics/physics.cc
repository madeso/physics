#include "physics.h"

#include <type_traits>
#include <cassert>
#include <cmath>


namespace physics
{

Collision NoCollision()
{
	return {};
}

Collision TestCollision
(
	const Shape& lhs_collider, const Transform& lhs_transform,
	const Shape& rhs_collider, const Transform& rhs_transform
);
 
void ResolveCollisions(World* world, float dt)
{
	auto SolveCollisions = [](const World& world, std::vector<CollisionBetween>& collisions, float dt)
	{
		for (const Solver& solve : world.solvers)
		{
			solve(collisions, dt);
		}
	};

	auto SendCollisionCallbacks = [](const World& world, std::vector<CollisionBetween>& collisions, float dt)
	{
		auto call = [](const OnCollision& cb, CollisionBetween& collision, float dt)
		{
			if (cb) { cb(collision, dt); }
		};

		for (CollisionBetween& collision : collisions)
		{
			call(world.on_collision, collision, dt);
			call(collision.a->on_collision, collision, dt);
			call(collision.b->on_collision, collision, dt);
		}
	};

	std::vector<CollisionBetween> collisions;
	std::vector<CollisionBetween> triggers;

	// todo(Gustav): reduce the N*M complexity
	for (Object* a : world->objects)
	{
		for (Object* b : world->objects)
		{
			if (a == b)
			{
				// todo(Gustav): is this properly iteration over all objects?
				break;
			}

			Collision collision = TestCollision(a->shape, a->transform, b->shape, b->transform);

			if (collision.has_collision)
			{
				const auto any_is_trigger = a->is_trigger || b->is_trigger;
				if (any_is_trigger)
				{
					triggers.emplace_back(CollisionBetween{a, b, collision});
				}
				else
				{
					collisions.emplace_back(CollisionBetween{a, b, collision});
				}
			}
		}
	}

	SolveCollisions(*world, collisions, dt);
	// Don't solve triggers

	SendCollisionCallbacks(*world, collisions, dt);
	SendCollisionCallbacks(*world, triggers, dt);
}
 
void ApplyGravity(World* world)
{
	if(world->apply_gravity == false)
	{
		return;
	}

	for (Object* object : world->objects)
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

void MoveObjects(World* world, float dt)
{
	for (Object* object : world->objects)
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

void Step(World* world, float dt)
{
	ApplyGravity(world);
	ResolveCollisions(world, dt);
	MoveObjects(world, dt);
}

Collision CreateCollision(const vector3& A, const vector3& B)
{
	const auto AtoB = B - A;
	const auto len = glm::length(AtoB);

	return
	{
		A, B,
		AtoB / len,
		len,
		true
	};
}

namespace algo
{
	Collision FindSphereSphereCollisionPoints(const Sphere& a, const Transform& ta, const Sphere& b, const Transform& tb)
	{
		const vector3 A = a.center + ta.position;
		const vector3 B = b.center + tb.position;

		const float Ar = a.radius * ta.scale;
		const float Br = b.radius * tb.scale;

		const vector3 AtoB = B - A;

		if (glm::length(AtoB) > Ar + Br)
		{
			return NoCollision();
		}

		const vector3 BtoA = A - B;
		const auto nA = A + glm::normalize(AtoB) * Ar;
		const auto nB = B + glm::normalize(BtoA) * Br;

		return CreateCollision(nA, nB);
	}

	Collision FindSpherePlaneCollisionPoints(const Sphere& a, const Transform& ta, const Plane& b, const Transform& tb)
	{
		assert(std::abs(glm::length2(b.normal) - 1) < 0.1f);
		const auto A  = a.center + ta.position;
		const auto Ar = a.radius * ta.scale;
		const auto N = glm::normalize(b.normal * tb.rotation);
		const auto P = N * b.distance + tb.position;

		// distance from center of sphere to plane surface
		const float d = glm::dot(A - P, N);

		if (d > Ar)
		{
			return NoCollision();
		}
		
		const auto B = A - N * d;
		const auto Ac = A - N * Ar;

		return CreateCollision(Ac, B);
	}
}

namespace forwarders
{
	Collision TestCollision(const Sphere& self, const Transform& transform, const Sphere& sphere, const Transform& sphereTransform)
	{
		return algo::FindSphereSphereCollisionPoints(self, transform, sphere, sphereTransform);
	}

	Collision TestCollision(const Sphere& self, const Transform& transform, const Plane& plane, const Transform& planeTransform)
	{
		return algo::FindSpherePlaneCollisionPoints(self, transform, plane, planeTransform);
	}

	Collision TestCollision(const Plane& self, const Transform& transform, const Sphere& sphere, const Transform& sphereTransform)
	{
		// reuse sphere code
		Collision collision = TestCollision(sphere, sphereTransform, self, transform);

		// You could have an algo Plane v Sphere to do the swap
		std::swap(collision.a, collision.b);
		collision.normal = -collision.normal;

		return collision;
	}

	Collision TestCollision(const Plane& self, const Transform& transform, const Plane& plane, const Transform& planeTransform)
	{
		return NoCollision();
	}
}

template<class> inline constexpr bool always_false_v = false;

namespace intermediate
{
	Collision TestCollision
	(
		const Sphere& lhs_collider, const Transform& lhs_transform,
		const Shape& rhs_collider, const Transform& rhs_transform
	)
	{
		return std::visit([&](const auto& rhs) -> Collision
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

	Collision TestCollision
	(
		const Plane& lhs_collider, const Transform& lhs_transform,
		const Shape& rhs_collider, const Transform& rhs_transform
	)
	{
		return std::visit([&](const auto& rhs) -> Collision
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

Collision TestCollision
(
	const Shape& lhs_collider, const Transform& lhs_transform,
	const Shape& rhs_collider, const Transform& rhs_transform
)
{
	return std::visit([&](const auto& lhs) -> Collision
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



void ImpulseSolver(std::vector<CollisionBetween>& collisions, float dt)
{
	for (CollisionBetween& coll : collisions)
	{
		// Replaces non dynamic objects with default values.

		auto& aBody = coll.a->body;
		auto& bBody = coll.b->body;

		glm::vec3 aVel = aBody ? aBody->velocity : glm::vec3(0.0f);
		glm::vec3 bVel = bBody ? bBody->velocity : glm::vec3(0.0f);
		glm::vec3 rVel = bVel - aVel;
		float  nSpd = glm::dot(rVel, coll.collision.normal);

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

		glm::vec3 impluse = j * coll.collision.normal;

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
		nSpd = glm::dot(rVel, coll.collision.normal);

		glm::vec3 tangent = rVel - nSpd * coll.collision.normal;

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


void PositionSolver(std::vector<CollisionBetween>& collisions, float dt)
{
	for (CollisionBetween& coll : collisions)
	{
		Object* aBody = coll.a;
		Object* bBody = coll.b;

		float aStatic = aBody->body.has_value() ? 0.0f : 1.0f;
		float bStatic = bBody->body.has_value() ? 0.0f : 1.0f;

		auto resolution = coll.collision.normal * coll.collision.depth / std::max(1.0f, aStatic + bStatic);

		aBody->transform.position += resolution * (1.0f - aStatic);
		bBody->transform.position -= resolution * (1.0f - bStatic);

		// clear forces if there is a collision
		if (aBody->body) aBody->body->force = glm::vec3{ 0,0,0 };
		if (bBody->body) bBody->body->force = glm::vec3{ 0,0,0 };
	}
}


}

