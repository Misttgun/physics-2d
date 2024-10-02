#include "Application.h"

#include "Graphics.h"
#include "physics/Constants.h"
#include "physics/Force.h"
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

	auto floor = RigidBody(BoxShape(Graphics::Width() - 50, 50), static_cast<float>(Graphics::Width()) / 2.0f, 
		static_cast<float>(Graphics::Height()) - 50, 0.0f);
	floor.m_restitution = 0.2f;
	m_world->AddBody(floor);

	auto leftWall = RigidBody(BoxShape(50, Graphics::Height() - 100), 50, static_cast<float>(Graphics::Height()) / 2.0f - 25, 0.0);
	leftWall.m_restitution = 0.2f;
	m_world->AddBody(leftWall);

	auto rightWall = RigidBody(BoxShape(50, Graphics::Height() - 100), static_cast<float>(Graphics::Width()) - 50, 
		static_cast<float>(Graphics::Height()) / 2.0f - 25, 0.0);
	rightWall.m_restitution = 0.2f;
	m_world->AddBody(rightWall);

	auto bigBox = RigidBody(BoxShape(200, 200), static_cast<float>(Graphics::Width()) / 2.0f, static_cast<float>(Graphics::Height()) / 2.0f, 0.0f);
	bigBox.m_rotation = 1.4f;
	bigBox.m_restitution = 0.5f;

	m_world->AddBody(bigBox);

	Vec2 wind(0.5f * PIXELS_PER_METER, 0.0f);
	m_world->AddForce(wind);
}

void Application::ProcessInput()
{
	if (IsKeyDown(KEY_D))
		m_debug = !m_debug;

	// if (IsKeyUp(KEY_UP) || IsKeyUp(KEY_DOWN))
	// 	m_pushForce.y = 0;
	// if (IsKeyUp(KEY_LEFT) || IsKeyUp(KEY_RIGHT))
	// 	m_pushForce.x = 0;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		const std::vector<Vec2> vertices = {
			Vec2(20, 60),
			Vec2(-40, 20),
			Vec2(-20, -60),
			Vec2(20, -60),
			Vec2(40, 20)
		};

		auto polygon = RigidBody(PolygonShape(vertices), static_cast<float>(GetMouseX()), static_cast<float>(GetMouseY()), 1.0f);
		polygon.m_restitution = 0.1f;
		polygon.m_friction = 0.7f;
		m_world->AddBody(polygon);
	}

	// if (IsMouseButtonUp(MOUSE_BUTTON_LEFT) && m_leftMouseButtonDown)
	// {
	// 	m_leftMouseButtonDown = false;
	// 	const Vec2 impulse = (m_rigidBodies[0]->position - m_mouseCursor);
	// 	const Vec2 impulse_direction = impulse.UnitVector();
	// 	const float impulse_magnitude = impulse.Magnitude();
	// 	m_rigidBodies[0]->velocity = impulse_direction * impulse_magnitude;
	// }

	// m_mouseCursor.x = GetMouseX();
	// m_mouseCursor.y = GetMouseY();

	// m_rigidBodies[0]->position.x = GetMouseX();
	// m_rigidBodies[0]->position.y = GetMouseY();
}

void Application::Update() const
{
	const float dt = GetFrameTime();

	m_world->Update(dt);
}

void Application::Render() const
{
	BeginDrawing();

	Graphics::ClearScreen(DARKGRAY);

	for (const RigidBody& body : m_world->GetBodies())
	{
		if (body.m_shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(body.m_shape.get());
			Graphics::DrawCircle(body.m_position, circleShape->m_radius, body.m_rotation, body.m_isColliding ? RED : WHITE);
		}

		if (body.m_shape->GetType() == BOX)
		{
			const BoxShape* boxShape = dynamic_cast<BoxShape*>(body.m_shape.get());
			Graphics::DrawPolygon(body.m_position, boxShape->m_worldVertices, body.m_isColliding ? RED : WHITE);
		}

		if (body.m_shape->GetType() == POLYGON)
		{
			const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(body.m_shape.get());
			Graphics::DrawPolygon(body.m_position, polygonShape->m_worldVertices, body.m_isColliding ? RED : WHITE);
		}
	}

	EndDrawing();
}

void Application::Destroy()
{
	Graphics::CloseWindow();
}
