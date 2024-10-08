#include "physics/World.h"
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
    for (const auto& body : m_bodies)
    {
        const Vec2 weight = Vec2(0.0f, m_gravity * PIXELS_PER_METER * body->m_mass);
        body->AddForce(weight);

        for(const auto &force : m_forces)
            body->AddForce(force);

        for(const auto torque : m_torques)
            body->AddTorque(torque);
    }

    for(const auto& body : m_bodies)
    {
	    body->Update(dt);
    }

	CheckCollisions();
}

void World::CheckCollisions() const
{
    Contact contact;

    // Check all the rigid bodies with the other rigid bodies for collision
	for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
	{
		for (std::size_t j = i + 1; j < m_bodies.size(); j++)
		{
			const auto a = m_bodies[i];
			const auto b = m_bodies[j];
			a->m_isColliding = false;
			b->m_isColliding = false;

			if (IsColliding(a, b, contact))
			{
                ResolveCollision(contact);
			}
		}
	}
}
