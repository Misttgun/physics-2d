#include "Application.h"

#include "Graphics.h"
#include "physics/RigidBody.h"
#include "physics/Constants.h"
#include "physics/Force.h"
#include "physics/Shape.h"
#include "physics/CollisionDetection.h"

bool Application::IsRunning() const
{
	return WindowShouldClose() == false;
}

void Application::Setup()
{
	Graphics::OpenWindow();
	SetTargetFPS(60);

	RigidBody* floor = new RigidBody(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2, Graphics::Height() - 50, 0.0f);
	floor->restitution = 0.2f;
	m_rigidBodies.push_back(floor);

	RigidBody* leftWall = new RigidBody(BoxShape(50, Graphics::Height() - 100), 50, Graphics::Height() / 2.0 - 25, 0.0);
	leftWall->restitution = 0.2;
	m_rigidBodies.push_back(leftWall);

	RigidBody* rightWall = new RigidBody(BoxShape(50, Graphics::Height() - 100), Graphics::Width() - 50, Graphics::Height() / 2.0 - 25, 0.0);
	rightWall->restitution = 0.2;
	m_rigidBodies.push_back(rightWall);

	RigidBody* bigBox = new RigidBody(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() / 2.0f, 0.0f);
	bigBox->rotation = 1.4f;
	bigBox->restitution = 0.5f;

	m_rigidBodies.push_back(bigBox);
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
		std::vector<Vec2> vertices = {Vec2(20, 60), Vec2(-40, 20), Vec2(-20, -60), Vec2(20, -60), Vec2(40, 20)};

		RigidBody* polygon = new RigidBody(PolygonShape(vertices), GetMouseX(), GetMouseY(), 1.0f);
		polygon->restitution = 0.1f;
		polygon->friction = 0.7f;
		m_rigidBodies.push_back(polygon);
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

	for (RigidBody* rigidBody : m_rigidBodies)
	{
		// rigidBody->AddForce(m_pushForce);

		// rigidBody->AddTorque(200);

		const Vec2 gravity = Vec2(0.0f, 9.8f * PIXELS_PER_METER * rigidBody->mass);
		rigidBody->AddForce(gravity);

		// const Vec2 wind = Vec2(20.0f * PIXELS_PER_METER, 0.0f);
		// rigidBody->AddForce(wind);

		/*const Vec2 drag = Force::GenerateDrag(*rigidBody, 0.01f);
		rigidBody->AddForce(drag);*/

		rigidBody->Update(dt);
	}

	// Check all the rigidbodies with the other rigidbodies for collision
	for (std::size_t i = 0; i < m_rigidBodies.size() - 1; i++)
	{
		for (std::size_t j = i + 1; j < m_rigidBodies.size(); j++)
		{
			RigidBody* a = m_rigidBodies[i];
			RigidBody* b = m_rigidBodies[j];
			a->isColliding = false;
			b->isColliding = false;

			Contact contact;

			if (CollisionDetection::IsColliding(a, b, contact))
			{
				contact.ResolveCollision();

				// Draw debug information
				if (debug)
				{
					Graphics::DrawFillCircle(contact.start, 3, PURPLE);
					Graphics::DrawFillCircle(contact.end, 3, PURPLE);
					Graphics::DrawLine(contact.start, contact.start + contact.normal * 15, PURPLE);

					a->isColliding = true;
					b->isColliding = true;
				}
			}
		}
	}
}

void Application::Render()
{
	BeginDrawing();

	Graphics::ClearScreen(DARKGRAY);

	for (const RigidBody* body : m_rigidBodies)
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
	for (const auto rbody : m_rigidBodies)
		delete rbody;

	Graphics::CloseWindow();
}
