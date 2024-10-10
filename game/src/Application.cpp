#include "Application.h"
#include "Graphics.h"
#include "physics/Constants.h"
#include "physics/Constraint.h"
#include "physics/RigidBody.h"
#include "physics/Shape.h"
#include "physics/World.h"

bool Application::IsRunning()
{
	return WindowShouldClose() == false;
}

void Application::Setup()
{
	Graphics::OpenWindow();
	SetTargetFPS(FPS);

	m_world = std::make_unique<World>(-9.8f);

	LoadResources();

	// Add bird
	const auto bird = std::make_shared<RigidBody>(CircleShape(30.0f), 100, Graphics::Height() - 180, 3.0f);
	bird->SetTexture("bird-red");
	m_world->AddBody(bird);

	// Add a floor and walls to contain objects
	const auto floor = std::make_shared<RigidBody>(BoxShape(Graphics::Width(), 50), Graphics::Width() / 2, Graphics::Height() - 125);
	const auto roof = std::make_shared<RigidBody>(BoxShape(Graphics::Width(), 50), Graphics::Width() / 2, -200);
	const auto leftFence = std::make_shared<RigidBody>(BoxShape(50, Graphics::Height() * 2), -25, Graphics::Height() / 2);
	const auto rightFence = std::make_shared<RigidBody>(BoxShape(50, Graphics::Height() * 2), Graphics::Width() + 25, Graphics::Height() / 2);
	m_world->AddBody(floor);
	m_world->AddBody(leftFence);
	m_world->AddBody(rightFence);
	m_world->AddBody(roof);

	// Add a stack of boxes
	for (int i = 1; i <= 4; i++)
	{
		float mass = 10.0f / static_cast<float>(i);
		const auto box = std::make_shared<RigidBody>(BoxShape(30, 30), 400, static_cast<int>(floor->m_position.y) - i * 40, mass);
		box->SetTexture("wood-box");
		box->m_friction = 0.9f;
		box->m_restitution = 0.1f;
		m_world->AddBody(box);
	}

	// Add structure with blocks
	const auto plank1 = std::make_shared<RigidBody>(BoxShape(30, 90), Graphics::Width() / 2 - 40, static_cast<int>(floor->m_position.y) - 70, 5.0f);
	const auto plank2 = std::make_shared<RigidBody>(BoxShape(30, 90), Graphics::Width() / 2 + 60, static_cast<int>(floor->m_position.y) - 70, 5.0f);
	const auto plank3 = std::make_shared<RigidBody>(BoxShape(180, 15), Graphics::Width() / 2 + 10, static_cast<int>(floor->m_position.y) - 130, 2.0f);
	plank1->SetTexture("wood-plank-solid");
	plank2->SetTexture("wood-plank-solid");
	plank3->SetTexture("wood-plank-cracked");
	m_world->AddBody(plank1);
	m_world->AddBody(plank2);
	m_world->AddBody(plank3);

	// Add a triangle polygon
	const std::vector<Vec2> triangleVertices = {
		Vec2(20, 20),
		Vec2(-20, 20),
		Vec2(0, -20)
	};

	const auto triangle = std::make_shared<RigidBody>(PolygonShape(triangleVertices), static_cast<int>(plank3->m_position.x), static_cast<int>(plank3->m_position.y) - 50, 0.5f);
	triangle->SetTexture("wood-triangle");
	m_world->AddBody(triangle);

	// Add a pyramid of boxes
	constexpr int numRows = 5;
	for (int col = 0; col < numRows; col++)
	{
		for (int row = 0; row < col; row++)
		{
			int x = static_cast<int>(plank3->m_position.x) + 200 + col * 33 - row * 17;
			int y = static_cast<int>(floor->m_position.y) - 50 - row * 52;
			float mass = 5.0f / (static_cast<float>(row) + 1.0f);
			const auto box = std::make_shared<RigidBody>(BoxShape(30, 30), x, y, mass);
			box->m_friction = 0.9f;
			box->m_restitution = 0.0f;
			box->SetTexture("wood-box");
			m_world->AddBody(box);
		}
	}

	// Add a bridge of connected steps and joints
	constexpr int numSteps = 10;
	constexpr int spacing = 20;
	const auto startStep = std::make_shared<RigidBody>(BoxShape(60, 15), 150, 150);
	startStep->SetTexture("rock-bridge-anchor");
	m_world->AddBody(startStep);

	auto last = floor;
	for (int i = 1; i <= numSteps; i++)
	{
		int x = static_cast<int>(startStep->m_position.x) + 20 + i * spacing;
		int y = static_cast<int>(startStep->m_position.y) + 15;
		float mass = i == numSteps ? 0.0f : 3.0f;
		const auto step = std::make_shared<RigidBody>(CircleShape(10.0f), x, y, mass);
		step->SetTexture("wood-bridge-step");
		m_world->AddBody(step);

		const auto joint = std::make_shared<JointConstraint>(last, step, step->m_position);
		m_world->AddConstraint(joint);
		last = step;
	}

	const auto endStep = std::make_shared<RigidBody>(BoxShape(60, 15), static_cast<int>(last->m_position.x) + 40, static_cast<int>(last->m_position.y) - 15);
	endStep->SetTexture("rock-bridge-anchor");
	m_world->AddBody(endStep);

	// Add pigs
	const auto pig1 = std::make_shared<RigidBody>(CircleShape(20.0f), static_cast<int>(plank1->m_position.x) + 50, static_cast<int>(floor->m_position.y) - 45, 3.0f);
	const auto pig2 = std::make_shared<RigidBody>(CircleShape(20.0f), static_cast<int>(plank2->m_position.x) + 400, static_cast<int>(floor->m_position.y) - 45, 3.0f);
	const auto pig3 = std::make_shared<RigidBody>(CircleShape(20.0f), static_cast<int>(pig2->m_position.x) + 40, static_cast<int>(floor->m_position.y) - 45, 3.0f);
	const auto pig4 = std::make_shared<RigidBody>(CircleShape(20.0f), 150, 100, 1.0f);
	pig1->SetTexture("pig-1");
	pig2->SetTexture("pig-2");
	pig3->SetTexture("pig-1");
	pig4->SetTexture("pig-2");
	m_world->AddBody(pig1);
	m_world->AddBody(pig2);
	m_world->AddBody(pig3);
	m_world->AddBody(pig4);
}

void Application::ProcessInput()
{
	if (IsKeyPressed(KEY_F2))
		m_debug = !m_debug;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		const auto circle = std::make_shared<RigidBody>(CircleShape(20.0f), GetMouseX(), GetMouseY(), 1.0f);
		circle->m_friction = 0.4f;
		circle->SetTexture("rock-round");
		m_world->AddBody(circle);
	}

	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
	{
		const auto box = std::make_shared<RigidBody>(BoxShape(40, 40), GetMouseX(), GetMouseY(), 1.0f);
		box->m_friction = 0.9f;
		box->m_angularVelocity = 0.0f;
		box->SetTexture("rock-box");
		m_world->AddBody(box);
	}

	if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))
	{
		m_world->GetBodies().at(0)->ApplyImpulseLinear(Vec2(-100.0f, 0.0f));
	}
	else if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))
	{
		m_world->GetBodies().at(0)->ApplyImpulseLinear(Vec2(100.0f, 0.0f));
	}

	if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W))
	{
		m_world->GetBodies().at(0)->ApplyImpulseLinear(Vec2(0.0f, -200.0f));
	}
}

void Application::Update() const
{
	float dt = GetFrameTime();

	// We tick physics on a fixed delta time
	if (dt > FIXED_DELTA_TIME)
		dt = FIXED_DELTA_TIME;

	m_world->Update(dt);
}

void Application::Render() const
{
	BeginDrawing();

	Graphics::ClearScreen({15, 7, 33, 255});

	if(true)
	{
		// Draw background texture
		Graphics::DrawTexture(Vec2(static_cast<float>(Graphics::Width()) / 2.0f, static_cast<float>(Graphics::Height()) / 2.0f),
							  static_cast<float>(Graphics::Width()), static_cast<float>(Graphics::Height()), 0.0f, m_resourceManager->GetTexture("background"));
	}

	const auto bodies = m_world->GetBodies();

	for (const auto& body : bodies)
	{
		if (body->m_shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(body->m_shape.get());
			if (m_debug == false && body->m_textureId.empty() == false)
			{
				Graphics::DrawTexture(body->m_position, circleShape->m_radius * 2, circleShape->m_radius * 2,
				                      body->m_rotation, m_resourceManager->GetTexture(body->m_textureId));
			}
			else if (m_debug)
			{
				Graphics::DrawCircle(body->m_position, circleShape->m_radius, body->m_rotation, GREEN);
			}
		}

		if (body->m_shape->GetType() == BOX)
		{
			const BoxShape* boxShape = dynamic_cast<BoxShape*>(body->m_shape.get());
			if (m_debug == false && body->m_textureId.empty() == false)
			{
				Graphics::DrawTexture(body->m_position, static_cast<float>(boxShape->m_width), static_cast<float>(boxShape->m_height),
				                      body->m_rotation, m_resourceManager->GetTexture(body->m_textureId));
			}
			else if (m_debug)
			{
				Graphics::DrawPolygon(body->m_position, boxShape->m_worldVertices, GREEN);
			}
		}

		if (body->m_shape->GetType() == POLYGON)
		{
			const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(body->m_shape.get());
			if (m_debug == false && body->m_textureId.empty() == false)
			{
				Graphics::DrawTexture(body->m_position, static_cast<float>(polygonShape->m_width), static_cast<float>(polygonShape->m_height),
				                      body->m_rotation, m_resourceManager->GetTexture(body->m_textureId));
			}
			else if (m_debug)
			{
				Graphics::DrawPolygon(body->m_position, polygonShape->m_worldVertices, GREEN);
			}
		}
	}

	EndDrawing();
}

void Application::Destroy()
{
	Graphics::CloseWindow();
}

void Application::LoadResources()
{
	SearchAndSetResourceDir("assets");

	m_resourceManager = std::make_unique<ResourceManager>();

	// Load resources
	m_resourceManager->AddTexture("background", "background.png");
	m_resourceManager->AddTexture("bird-red", "bird-red.png");
	m_resourceManager->AddTexture("pig-1", "pig-1.png");
	m_resourceManager->AddTexture("pig-2", "pig-2.png");
	m_resourceManager->AddTexture("rock-box", "rock-box.png");
	m_resourceManager->AddTexture("rock-bridge-anchor", "rock-bridge-anchor.png");
	m_resourceManager->AddTexture("rock-round", "rock-round.png");
	m_resourceManager->AddTexture("wood-box", "wood-box.png");
	m_resourceManager->AddTexture("wood-bridge-step", "wood-bridge-step.png");
	m_resourceManager->AddTexture("wood-plank-cracked", "wood-plank-cracked.png");
	m_resourceManager->AddTexture("wood-plank-solid", "wood-plank-solid.png");
	m_resourceManager->AddTexture("wood-triangle", "wood-triangle.png");
}
