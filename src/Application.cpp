#include "Application.h"

#include "Graphics.h"
#include "physics/RigidBody.h"
#include "physics/Constants.h"
#include "physics/Force.h"
#include "physics/Shape.h"
#include "physics/CollisionDetection.h"
#include "physics/World.h"

bool Application::IsRunning() const
{
	return WindowShouldClose() == false;
}

void Application::Setup()
{
	Graphics::OpenWindow();
	SetTargetFPS(60);

	m_world = new World(-9.8);

	RigidBody* floor = new RigidBody(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2, Graphics::Height() - 50, 0.0f);
	floor->restitution = 0.2f;
	m_world->AddBody(floor);

	RigidBody* leftWall = new RigidBody(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2.0 - 25, 0.0);
	leftWall->restitution = 0.2;
	m_world->AddBody(leftWall);

	RigidBody* rightWall = new RigidBody(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2.0 - 25, 0.0);
	rightWall->restitution = 0.2;
	m_world->AddBody(rightWall);

	RigidBody* bigBox = new RigidBody(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() / 2.0f, 0.0f);
	bigBox->rotation = 1.4f;
	bigBox->restitution = 0.5f;

	m_world->AddBody(bigBox);

	Vec2 wind(0.5f * PIXELS_PER_METER, 0.0f);
	m_world->AddForce(wind);
}

void Application::ProcessInput()
{
	if (IsKeyDown(KEY_D))
		debug = !debug;

	// if (IsKeyUp(KEY_UP) || IsKeyUp(KEY_DOWN))
	// 	m_pushForce.y = 0;
	// if (IsKeyUp(KEY_LEFT) || IsKeyUp(KEY_RIGHT))
	// 	m_pushForce.x = 0;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		std::vector<Vec2> vertices = { Vec2(20, 60), Vec2(-40, 20), Vec2(-20, -60), Vec2(20, -60), Vec2(40, 20) };

		RigidBody* polygon = new RigidBody(PolygonShape(vertices), GetMouseX(), GetMouseY(), 1.0f);
		polygon->restitution = 0.1f;
		polygon->friction = 0.7f;
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

void Application::Update()
{
	float dt = GetFrameTime();

	m_world->Update(dt);
}

void Application::Render()
{
	BeginDrawing();

	Graphics::ClearScreen(DARKGRAY);

	for (const RigidBody* body : m_world->GetBodies())
	{
		if (body->shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(body->shape);
			Graphics::DrawCircle(body->position, circleShape->radius, body->rotation, body->isColliding ? RED : WHITE);
		}

		if (body->shape->GetType() == BOX)
		{
			const BoxShape* boxShape = dynamic_cast<BoxShape*>(body->shape);
			Graphics::DrawPolygon(body->position, boxShape->worldVertices, body->isColliding ? RED : WHITE);
		}

		if (body->shape->GetType() == POLYGON)
		{
			const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(body->shape);
			Graphics::DrawPolygon(body->position, polygonShape->worldVertices, body->isColliding ? RED : WHITE);
		}
	}

	EndDrawing();
}

void Application::Destroy()
{
	delete m_world;

	Graphics::CloseWindow();
}
