#pragma once

#include <vector>

#include "physics/Vec2.h"

struct RigidBody;

class Application
{
private:
	std::vector<RigidBody*> m_rigidBodies;

	// Inputs
	Vec2 m_pushForce = Vec2::Zero();
	Vec2 m_mouseCursor = Vec2::Zero();
	bool m_leftMouseButtonDown = false;

public:
	Application() = default;
	~Application() = default;
	bool IsRunning() const;
	void Setup();
	void ProcessInput();
	void Update();
	void Render();
	void Destroy();
};
