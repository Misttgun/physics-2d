#pragma once

#include <vector>

#include "physics/Vec2.h"

struct RigidBody;

class Application
{
private:
	std::vector<RigidBody*> m_rigidBodies;
	bool debug = false;

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
