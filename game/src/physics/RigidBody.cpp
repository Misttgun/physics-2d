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

	m_shape->UpdateVertices(m_position, m_rotation);
	UpdateBoundingRadius();
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

Vec2 RigidBody::LocalToWorld(const Vec2& point) const
{
	const Vec2 rotated = point.Rotate(m_rotation);
	return rotated + m_position;
}

Vec2 RigidBody::WorldToLocal(const Vec2& point) const
{
	const float translatedX = point.x - m_position.x;
	const float translatedY = point.y - m_position.y;
	const float rotatedX = cos(-m_rotation) * translatedX - sin(-m_rotation) * translatedY;
	const float rotatedY = cos(-m_rotation) * translatedY + sin(-m_rotation) * translatedX;

	return {rotatedX, rotatedY};
}

void RigidBody::ClearForces()
{
	m_sumForces = Vec2(0.0f, 0.0f);
}

void RigidBody::ApplyImpulseLinear(const Vec2& j)
{
	if (IsStatic())
		return;

	m_velocity += j * m_invMass;
}

void RigidBody::ApplyImpulseAngular(const float j)
{
	if (IsStatic())
		return;

	m_angularVelocity += j * m_invInertia;
}

void RigidBody::ApplyImpulseAtPoint(const Vec2& j, const Vec2& r)
{
	if (IsStatic())
		return;

	m_velocity += j * m_invMass;
	m_angularVelocity += r.Cross(j) * m_invInertia;
}

void RigidBody::IntegrateForces(const float dt)
{
	if (IsStatic())
		return;

	m_acceleration = m_sumForces * m_invMass;
	m_velocity += m_acceleration * dt;

	m_angularAcceleration = m_sumTorque * m_invInertia;
	m_angularVelocity += m_angularAcceleration * dt;

	ClearForces();
	ClearTorque();
}

void RigidBody::IntegrateVelocities(const float dt)
{
	if (IsStatic())
		return;

	m_position += (m_velocity * dt);
	m_rotation += m_angularVelocity * dt;

	m_shape->UpdateVertices(m_position, m_rotation);
}

void RigidBody::SetTexture(const std::string& textureId)
{
	m_textureId = textureId;
}

void RigidBody::UpdateBoundingRadius()
{
	if (m_shape->GetType() == CIRCLE)
	{
		const auto* circleShape = dynamic_cast<CircleShape*>(m_shape.get());
		m_radius = circleShape->m_radius;
	}
	else if (m_shape->GetType() == POLYGON || m_shape->GetType() == BOX)
	{
		const auto* polygonShape = dynamic_cast<PolygonShape*>(m_shape.get());
		float maxDistance = 0.0f;
		
		// Find the furthest vertex from the center
		for (const auto& vertex : polygonShape->m_worldVertices)
		{
			const float distance = (vertex - m_position).MagnitudeSquared();
			maxDistance = std::max(maxDistance, distance);
		}
		
		m_radius = sqrt(maxDistance);
	}
}
