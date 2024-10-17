#pragma once

#include <vector>

#include "Vec2.h"

struct JointConstraint;
class RigidBody;

class World
{
private:
	float m_gravity;
	std::vector<RigidBody*> m_bodies;
	std::vector<JointConstraint*> m_constraints;
	std::vector<Vec2> m_forces;
	std::vector<float> m_torques;

public:
	explicit World(float gravity);

	void AddBody(RigidBody* body);
	std::vector<RigidBody*>& GetBodies();

	void AddConstraint(JointConstraint* constraint);
	std::vector<JointConstraint*>& GetConstraints();

	void AddForce(const Vec2& force);
	void AddTorque(float torque);

	void Update(float dt) const;
};
