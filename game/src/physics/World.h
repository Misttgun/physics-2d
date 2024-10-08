#pragma once

#include <vector>

#include "RigidBody.h"
#include "Vec2.h"

#include <memory>

class World
{
    private:
        float m_gravity;
        std::vector<std::shared_ptr<RigidBody>> m_bodies;
        std::vector<Vec2> m_forces;
        std::vector<float> m_torques;

    public:
        explicit World(float gravity);

        void AddBody(const std::shared_ptr<RigidBody>& body);
        std::vector<std::shared_ptr<RigidBody>>& GetBodies();

        void AddForce(const Vec2& force);
        void AddTorque(float torque);

        void Update(float dt) const;

        void CheckCollisions() const;
};