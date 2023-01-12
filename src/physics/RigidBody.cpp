#include "RigidBody.h"
#include "Shape.h"

#include <cassert>
#include <math.h>

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

	restitution = 1.0f;
	friction = 0.7f;

	mass = mass_;
	if (mass != 0.0f)
		invMass = 1 / mass;
	else
		invMass = 0.0f;

	inertia = shape->GetMomentOfInertia() * mass;
	if (inertia != 0.0f)
		invInertia = 1 / inertia;
	else
		invInertia = 0.0f;
}

RigidBody::~RigidBody()
{
	delete shape;
}

bool RigidBody::IsStatic() const
{
	return fabs(invMass - 0) < 0.005f;
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

void RigidBody::ApplyImpulse(const Vec2& impulse)
{
	if (IsStatic())
		return;

	velocity += impulse * invMass;
}

void RigidBody::ApplyImpulse(const Vec2& impulse, const Vec2& r)
{
	if(IsStatic())
		return;
	
	velocity += impulse * invMass;
	angularVelocity += r.Cross(impulse) * invInertia;
}

void RigidBody::IntegrateLinear(const float dt)
{
	if (IsStatic())
		return;

	acceleration = sumForces * invMass;

	velocity += (acceleration * dt);
	position += (velocity * dt);

	ClearForces();
}

void RigidBody::IntegrateAngular(const float dt)
{
	if (IsStatic())
		return;

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
