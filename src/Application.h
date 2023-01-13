#pragma once

#include <vector>

#include "physics/Vec2.h"

struct World;

class Application
{
private:
	World* m_world;
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
