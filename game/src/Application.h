#pragma once

#include <memory>
#include "ResourcesManager.h"
#include "memory/Arena.h"
#include "physics/Shape.h"
#include "physics/World.h"

struct JointConstraint;
class RigidBody;

class Application
{
private:
	std::unique_ptr<World> m_world;
	std::unique_ptr<ResourceManager> m_resourceManager;
	bool m_debug = false;

	Arena m_rbArena;
	Arena m_constraintArena;

public:
	Application() = default;
	[[nodiscard]] static bool IsRunning();
	void Setup();
	void ProcessInput();
	void Update() const;
	void Render() const;
	void Destroy();

private:
	void LoadResources();
	RigidBody* CreateRigidBody(const Shape& shape, const int x, const int y, const float mass = 0.0f);
	JointConstraint* CreateJointConstraint(RigidBody* aRb, RigidBody* bRb, const Vec2& anchorPoint);
};
