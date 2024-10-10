#include "physics/World.h"
#include "Constraint.h"
#include "physics/CollisionDetection.h"
#include "physics/Constants.h"
#include "physics/RigidBody.h"

World::World(const float gravity)
{
	m_gravity = -gravity;
}

void World::AddBody(const std::shared_ptr<RigidBody>& body)
{
	m_bodies.push_back(body);
}

std::vector<std::shared_ptr<RigidBody>>& World::GetBodies()
{
	return m_bodies;
}

void World::AddConstraint(const std::shared_ptr<Constraint>& constraint)
{
	m_constraints.push_back(constraint);
}

std::vector<std::shared_ptr<Constraint>>& World::GetConstraints()
{
	return m_constraints;
}

void World::AddForce(const Vec2& force)
{
	m_forces.emplace_back(force.x, force.y);
}

void World::AddTorque(const float torque)
{
	m_torques.emplace_back(torque);
}

void World::Update(const float dt) const
{
	// Create a vector of penetration constraints that will be solved frame per frame
	std::vector<PenetrationConstraint> penetrations;

	for (const auto& body : m_bodies)
	{
		const Vec2 weight = Vec2(0.0f, m_gravity * PIXELS_PER_METER * body->m_mass);
		body->AddForce(weight);

		for (const auto& force : m_forces)
			body->AddForce(force);

		for (const auto torque : m_torques)
			body->AddTorque(torque);
	}

	// Integrate all the forces
	for (const auto& body : m_bodies)
		body->IntegrateForces(dt);

	// Check all the rigid bodies with the other rigid bodies for collision
	for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
	{
		for (std::size_t j = i + 1; j < m_bodies.size(); j++)
		{
			std::vector<Contact> contacts;
			if (IsColliding(m_bodies[i], m_bodies[j], contacts))
			{
				for (const auto& contact : contacts) 
					penetrations.emplace_back(contact.a, contact.b, contact.start, contact.end, contact.normal);
			}
		}
	}

	// Solve all constraints
	for (const auto& constraint : m_constraints)
		constraint->PreSolve(dt);

	for (auto& constraint : penetrations)
		constraint.PreSolve(dt);

	for (int i = 0; i < 10; ++i)
	{
		for (const auto& constraint : m_constraints)
			constraint->Solve();

		for (auto& constraint : penetrations)
			constraint.Solve();
	}

	for (const auto& constraint : m_constraints)
		constraint->PostSolve();

	for (auto& constraint : penetrations)
		constraint.PostSolve();

	// Integrate all the velocities
	for (const auto& body : m_bodies)
		body->IntegrateVelocities(dt);
}
