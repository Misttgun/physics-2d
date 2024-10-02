#pragma once

#include <memory>
#include "physics/World.h"

class Application
{
private:
	std::unique_ptr<World> m_world;
	bool m_debug = false;

public:
	Application() = default;
	[[nodiscard]] static bool IsRunning();
	void Setup();
	void ProcessInput();
	void Update() const;
	void Render() const;
	static void Destroy();
};
