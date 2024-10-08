#include "Application.h"

#include "Graphics.h"
#include "physics/Constants.h"
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

	SearchAndSetResourceDir("assets");

	m_resourceManager = std::make_unique<ResourceManager>();

	// Load resources
	m_resourceManager->AddTexture("basketball-image", "basketball.png");
	m_resourceManager->AddTexture("bowlingball-image", "bowlingball.png");
	m_resourceManager->AddTexture("crate-image", "crate.png");
	m_resourceManager->AddTexture("metal-image", "metal.png");

	m_world = std::make_unique<World>(-9.8f);

	const auto floor = std::make_shared<RigidBody>(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2, Graphics::Height() - 50, 0.0f);
	floor->m_restitution = 0.2f;
	floor->SetTexture("metal-image");
	m_world->AddBody(floor);

	const auto leftWall = std::make_shared<RigidBody>(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2 - 25, 0.0f);
	leftWall->m_restitution = 0.2f;
	leftWall->SetTexture("metal-image");
	m_world->AddBody(leftWall);

	const auto rightWall = std::make_shared<RigidBody>(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2 - 25, 0.0f);
	rightWall->m_restitution = 0.2f;
	rightWall->SetTexture("metal-image");
	m_world->AddBody(rightWall);

	const auto bigBox = std::make_shared<RigidBody>(BoxShape(200, 200), Graphics::Width() / 2, Graphics::Height() / 2, 0.0f);
	bigBox->m_rotation = 1.4f;
	bigBox->m_restitution = 0.5f;
	bigBox->SetTexture("crate-image");
	m_world->AddBody(bigBox);

	const Vec2 wind(0.5f * PIXELS_PER_METER, 0.0f);
	m_world->AddForce(wind);
}

void Application::ProcessInput()
{
	if (IsKeyPressed(KEY_D))
		m_debug = !m_debug;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		const auto circle = std::make_shared<RigidBody>(CircleShape(30.0f), GetMouseX(), GetMouseY(), 1.0f);
		circle->m_restitution = 0.5f;
		circle->SetTexture("basketball-image");
		m_world->AddBody(circle);
	}

	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
	{
		const auto box = std::make_shared<RigidBody>(BoxShape(60, 60), GetMouseX(), GetMouseY(), 1.0f);
		box->m_restitution = 0.1f;
		box->SetTexture("crate-image");
		m_world->AddBody(box);
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

	Graphics::ClearScreen(BLACK);

	for (const auto& body : m_world->GetBodies())
	{
		if (body->m_shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(body->m_shape.get());
			if (m_debug == false && body->m_textureId.empty() == false)
			{
				Graphics::DrawTexture(body->m_position, circleShape->m_radius * 2, circleShape->m_radius * 2,
				                      body->m_rotation, m_resourceManager->GetTexture(body->m_textureId));
			}
			else
			{
				Graphics::DrawCircle(body->m_position, circleShape->m_radius, body->m_rotation, body->m_isColliding ? RED : WHITE);
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
			else
			{
				Graphics::DrawPolygon(body->m_position, boxShape->m_worldVertices, body->m_isColliding ? RED : WHITE);
			}
		}

		if (body->m_shape->GetType() == POLYGON)
		{
			const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(body->m_shape.get());
			if (m_debug == false)
			{
				Graphics::DrawFillPolygon(body->m_position, polygonShape->m_worldVertices, WHITE);
			}
			else
			{
				Graphics::DrawPolygon(body->m_position, polygonShape->m_worldVertices, body->m_isColliding ? RED : WHITE);
			}
		}
	}

	EndDrawing();
}

void Application::Destroy()
{
	Graphics::CloseWindow();
}
