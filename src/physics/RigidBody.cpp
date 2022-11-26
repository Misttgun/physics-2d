#include "RigidBody.h"
#include "Shape.h"

#include <cassert>

RigidBody::RigidBody(const Shape& shape_, float x, float y, float mass_)
{
	shape = shape_.Clone();
	position = Vec2(x, y);

	velocity = Vec2::Zero();
	acceleration = Vec2::Zero();

	rotation = 0.0f;
	angularAcceleration = 0.0f;
	angularVelocity = 0.0f;

	sumForces = Vec2::Zero();
	sumTorque = 0.0f;

	mass = mass_;
	assert(mass != 0.0f);
	invMass = 1 / mass;

	inertia = shape->GetMomentOfInertia() * mass;
	assert(inertia != 0.0f);
	invInertia = 1 / inertia;
}

RigidBody::~RigidBody()
{
	delete shape;
}

void RigidBody::AddForce(const Vec2& force)
{
	sumForces += force;
}

void RigidBody::AddTorque(const float torque)
{
	sumTorque += torque;
}

void RigidBody::ClearTorque()
{
	sumTorque = 0.0f;
}

void RigidBody::ClearForces()
{
	sumForces = Vec2(0.0f, 0.0f);
}

void RigidBody::IntegrateLinear(const float dt)
{
	acceleration = sumForces * invMass;

	velocity += (acceleration * dt);
	position += (velocity * dt);

	ClearForces();
}

void RigidBody::IntegrateAngular(const float dt)
{
	angularAcceleration = sumTorque * invInertia;

	angularVelocity += angularAcceleration * dt;
	rotation += angularVelocity * dt;

	ClearTorque();
}

void RigidBody::Update(const float dt)
{
	IntegrateLinear(dt);
	IntegrateAngular(dt);

	if (shape->GetType() == BOX || shape->GetType() == POLYGON)
	{
		PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(shape);
		polygonShape->UpdateVertices(position, rotation);
	}
}
