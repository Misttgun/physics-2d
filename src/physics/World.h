#pragma once

#include <vector>
#include "Vec2.h"

struct RigidBody;


class World
{
    private:
        float m_gravity;
        std::vector<RigidBody*> m_bodies;
        std::vector<Vec2> m_forces;
        std::vector<float> m_torques;

    public:
        World(float gravity);
        ~World();

        void AddBody(RigidBody* body);
        std::vector<RigidBody*>& GetBodies();

        void AddForce(const Vec2& force);
        void AddTorque(float torque);

        void Update(float dt);

        void CheckCollisions();
};