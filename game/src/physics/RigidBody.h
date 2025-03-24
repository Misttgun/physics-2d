#pragma once

#include <memory>
#include <string>

#include "Shape.h"
#include "Vec2.h"

class RigidBody
{
public:
	// Mass properties
	float m_mass;
	float m_invMass;
	float m_inertia;
	float m_invInertia;

	// Physical properties
	float m_restitution;
	float m_friction;

	// Linear motion
	Vec2 m_position;
	Vec2 m_velocity;
	Vec2 m_acceleration;
	Vec2 m_sumForces;

	// Angular motion
	float m_rotation;
	float m_angularVelocity;
	float m_angularAcceleration;
	float m_sumTorque;

	// Broadphase
	float m_radius; // Circle radius for the broadphase check

	// Dynamic allocations
	std::unique_ptr<Shape> m_shape;
	std::string m_textureId;

	RigidBody(const Shape& shape, int x, int y, float mass = 0.0f);

	[[nodiscard]] bool IsStatic() const;

	void AddForce(const Vec2& force);
	void AddTorque(float torque);
	void ClearForces();
	void ClearTorque();

	[[nodiscard]] Vec2 LocalToWorld(const Vec2& point) const;
	[[nodiscard]] Vec2 WorldToLocal(const Vec2& point) const;

	void ApplyImpulseLinear(const Vec2& j);
	void ApplyImpulseAngular(float j);
	void ApplyImpulseAtPoint(const Vec2& j, const Vec2& r);

	void IntegrateForces(float dt);
	void IntegrateVelocities(float dt);

	void SetTexture(const std::string& textureId);

	// Broad phase methods
	void UpdateBoundingRadius();
};
