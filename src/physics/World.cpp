#include "World.h"
#include "RigidBody.h"
#include "Constants.h"
#include "CollisionDetection.h"

World::World(float gravity)
{
    m_gravity = -gravity;
}

World::~World()
{
    for (auto body : m_bodies)
        delete body;
}

void World::AddBody(RigidBody* body)
{
    m_bodies.push_back(body);
}

std::vector<RigidBody*>& World::GetBodies()
{
    return m_bodies;
}

void World::AddForce(const Vec2& force)
{
    m_forces.push_back(force);
}

void World::AddTorque(float torque)
{
    m_torques.push_back(torque);
}

void World::Update(float dt)
{
    for (auto body : m_bodies)
    {
        const Vec2 weight = Vec2(0.0f, m_gravity * PIXELS_PER_METER * body->mass);
        body->AddForce(weight);

        for(auto force : m_forces)
            body->AddForce(force);

        for(auto torque : m_torques)
            body->AddTorque(torque);
    }

    for(auto body : m_bodies)
        body->Update(dt);

    CheckCollisions();
}

void World::CheckCollisions()
{
    // Check all the rigidbodies with the other rigidbodies for collision
	for (std::size_t i = 0; i < m_bodies.size() - 1; i++)
	{
		for (std::size_t j = i + 1; j < m_bodies.size(); j++)
		{
			RigidBody* a = m_bodies[i];
			RigidBody* b = m_bodies[j];
			a->isColliding = false;
			b->isColliding = false;

			Contact contact;

			if (CollisionDetection::IsColliding(a, b, contact))
			{
				contact.ResolveCollision();
			}
		}
	}
}