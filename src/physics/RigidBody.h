#pragma once

#include "Vec2.h"

struct Shape;

struct RigidBody
{
	bool isColliding = false;

	// Linear motion
	Vec2 position;
	Vec2 velocity;
	Vec2 acceleration;

	// Angular motion
	float rotation;
	float angularVelocity;
	float angularAcceleration;

	// Forces and Torque
	Vec2 sumForces;
	float sumTorque;

	// Mass and Moment of Inertia
	float mass;
	float invMass;
	float inertia;
	float invInertia;

	// Coefficient of restitution (elasticity)
	float restitution;

	// Pointer to shape
	Shape* shape;

	RigidBody(const Shape& shape_, float x, float y, float mass_);
	~RigidBody();

	bool IsStatic() const;

	void AddForce(const Vec2& force);
	void AddTorque(float torque);
	void ClearForces();
	void ClearTorque();

	void ApplyImpulse(const Vec2& impulse);

	void IntegrateLinear(float dt);
	void IntegrateAngular(float dt);

	void Update(float dt);
};
