#include "physics/RigidBody.h"
#include "physics/Shape.h"

#include <cmath>

RigidBody::RigidBody(const Shape& shape, const int x, const int y, const float mass)
{
	m_shape = shape.Clone();
	m_position = Vec2(static_cast<float>(x), static_cast<float>(y));

	m_velocity = Vec2::Zero();
	m_acceleration = Vec2::Zero();

	m_rotation = 0.0f;
	m_angularAcceleration = 0.0f;
	m_angularVelocity = 0.0f;

	m_sumForces = Vec2::Zero();
	m_sumTorque = 0.0f;

	m_restitution = 1.0f;
	m_friction = 0.7f;

	m_mass = mass;
	if (m_mass != 0.0f)
		m_invMass = 1 / m_mass;
	else
		m_invMass = 0.0f;

	m_inertia = m_shape->GetMomentOfInertia() * m_mass;
	if (m_inertia != 0.0f)
		m_invInertia = 1 / m_inertia;
	else
		m_invInertia = 0.0f;
}

bool RigidBody::IsStatic() const
{
	return fabs(m_invMass - 0) < 0.005f;
}

void RigidBody::AddForce(const Vec2& force)
{
	m_sumForces += force;
}

void RigidBody::AddTorque(const float torque)
{
	m_sumTorque += torque;
}

void RigidBody::ClearTorque()
{
	m_sumTorque = 0.0f;
}

void RigidBody::ClearForces()
{
	m_sumForces = Vec2(0.0f, 0.0f);
}

void RigidBody::ApplyImpulse(const Vec2& impulse)
{
	if (IsStatic())
		return;

	m_velocity += impulse * m_invMass;
}

void RigidBody::ApplyImpulse(const Vec2& impulse, const Vec2& r)
{
	if (IsStatic())
		return;

	m_velocity += impulse * m_invMass;
	m_angularVelocity += r.Cross(impulse) * m_invInertia;
}

void RigidBody::IntegrateLinear(const float dt)
{
	if (IsStatic())
		return;

	m_acceleration = m_sumForces * m_invMass;

	m_velocity += (m_acceleration * dt);
	m_position += (m_velocity * dt);

	ClearForces();
}

void RigidBody::IntegrateAngular(const float dt)
{
	if (IsStatic())
		return;

	m_angularAcceleration = m_sumTorque * m_invInertia;

	m_angularVelocity += m_angularAcceleration * dt;
	m_rotation += m_angularVelocity * dt;

	ClearTorque();
}

void RigidBody::Update(const float dt)
{
	IntegrateLinear(dt);
	IntegrateAngular(dt);
	m_shape->UpdateVertices(m_position, m_rotation);
}

void RigidBody::SetTexture(const std::string& textureId)
{
	m_textureId = textureId;
}
