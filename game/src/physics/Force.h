#pragma once

#include "Vec2.h"
#include "RigidBody.h"
#include <algorithm>

struct Force
{
	static Vec2 GenerateDrag(const RigidBody& particle, float k);
	static Vec2 GenerateFriction(const RigidBody& particle, float k);
	static Vec2 GenerateAttraction(const RigidBody& a, const RigidBody& b, float g);
	static Vec2 GenerateSpring(const RigidBody& particle, const Vec2& anchor, float rest_length, float k);
};

inline Vec2 Force::GenerateDrag(const RigidBody& particle, const float k)
{
	Vec2 drag = Vec2::Zero();

	if (particle.velocity.MagnitudeSquared() > 0)
	{
		const Vec2 direction = particle.velocity.Normalized() * -1.0f;
		const float magnitude = k * particle.velocity.MagnitudeSquared();
		drag = direction * magnitude;
	}

	return drag;
}

inline Vec2 Force::GenerateFriction(const RigidBody& particle, const float k)
{
	Vec2 friction = Vec2::Zero();

	const Vec2 direction = particle.velocity.UnitVector() * -1;
	const float magnitude = k;
	friction = direction * magnitude;

	return friction;
}

/*
 * Generate gravitational attraction Force that b has over a.
 * a will be pulled towards b. 
 */
inline Vec2 Force::GenerateAttraction(const RigidBody& a, const RigidBody& b, float g)
{
	const Vec2 direction = b.position - a.position;
	const Vec2 attraction_dir = direction.UnitVector();

	float distance_squared = direction.MagnitudeSquared();
	distance_squared = std::clamp(distance_squared, 5.0f, 100.0f);

	const float attraction_magnitude = g * (a.mass * b.mass) / distance_squared;

	return attraction_dir * attraction_magnitude;
}

inline Vec2 Force::GenerateSpring(const RigidBody& particle, const Vec2& anchor, const float rest_length, const float k)
{
	const Vec2 direction = particle.position - anchor;
	const float displacement = direction.Magnitude() - rest_length;
	const Vec2 spring_direction = direction.UnitVector();
	const float spring_magnitude = -k * displacement;

	return spring_direction * spring_magnitude;
}
