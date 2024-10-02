#pragma once

#include <vector>

#include "RigidBody.h"
#include "Vec2.h"


class World
{
    private:
        float m_gravity;
        std::vector<RigidBody> m_bodies;
        std::vector<Vec2> m_forces;
        std::vector<float> m_torques;

    public:
        explicit World(float gravity);

        void AddBody(const RigidBody& body);
        std::vector<RigidBody>& GetBodies();

        void AddForce(const Vec2& force);
        void AddTorque(float torque);

        void Update(float dt);

        void CheckCollisions();
};