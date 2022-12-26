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

	RigidBody* boxA = new RigidBody(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() /2.0f, 1.0f);
	boxA->angularVelocity = 0.4f;
	
	RigidBody* boxB = new RigidBody(BoxShape(200, 200), Graphics::Width() / 2.0f, Graphics::Height() /2.0f, 1.0f);
	boxB->angularVelocity = 0.1f;

	m_rigidBodies.push_back(boxA);
	m_rigidBodies.push_back(boxB);
}

void Application::ProcessInput()
{
	// if (IsKeyDown(KEY_UP))
	// 	m_pushForce.y = -50 * PIXELS_PER_METER;
	// if (IsKeyDown(KEY_DOWN))
	// 	m_pushForce.y = 50 * PIXELS_PER_METER;
	// if (IsKeyDown(KEY_LEFT))
	// 	m_pushForce.x = -50 * PIXELS_PER_METER;
	// if (IsKeyDown(KEY_RIGHT))
	// 	m_pushForce.x = 50 * PIXELS_PER_METER;

	// if (IsKeyUp(KEY_UP) || IsKeyUp(KEY_DOWN))
	// 	m_pushForce.y = 0;
	// if (IsKeyUp(KEY_LEFT) || IsKeyUp(KEY_RIGHT))
	// 	m_pushForce.x = 0;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		RigidBody* smallBall = new RigidBody(CircleShape(40), GetMouseX(), GetMouseY(), 1.0f);
		smallBall->restitution = 0.9f;
		m_rigidBodies.push_back(smallBall);
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

	m_rigidBodies[0]->position.x = GetMouseX();
	m_rigidBodies[0]->position.y = GetMouseY();
}

void Application::Update()
{
	// static int time_prev_frame;
	// const uint32_t curr_time = SDL_GetTicks();
	// const int32_t time_to_wait = MILLISECS_PER_FRAME - (curr_time - time_prev_frame);
	// if (time_to_wait > 0)
	// 	SDL_Delay(time_to_wait);

	// float dt = (curr_time - time_prev_frame) / 1000.0F;
	// if (dt > 0.016f)
	// 	dt = 0.016f;

	// time_prev_frame = curr_time;

	float dt = GetFrameTime();

	for (RigidBody* rigidBody : m_rigidBodies)
	{
		// rigidBody->AddForce(m_pushForce);

		// rigidBody->AddTorque(200);

		// const Vec2 gravity = Vec2(0.0f, 9.8f * PIXELS_PER_METER * rigidBody->mass);
		// rigidBody->AddForce(gravity);

		// const Vec2 wind = Vec2(20.0f * PIXELS_PER_METER, 0.0f);
		// rigidBody->AddForce(wind);

		/*const Vec2 drag = Force::GenerateDrag(*rigidBody, 0.01f);
		rigidBody->AddForce(drag);*/

		rigidBody->Update(dt);

		rigidBody->isColliding = false;

		if (rigidBody->shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(rigidBody->shape);

			if (rigidBody->position.y + circleShape->radius >= Graphics::Height())
			{
				rigidBody->position.y = Graphics::Height() - circleShape->radius;
				rigidBody->velocity.y *= -0.8f;
			}
			else if (rigidBody->position.y - circleShape->radius <= 0)
			{
				rigidBody->position.y = circleShape->radius;
				rigidBody->velocity.y *= -0.8f;
			}

			if (rigidBody->position.x + circleShape->radius >= Graphics::Width())
			{
				rigidBody->position.x = Graphics::Width() - circleShape->radius;
				rigidBody->velocity.x *= -0.8f;
			}
			else if (rigidBody->position.x - circleShape->radius <= 0)
			{
				rigidBody->position.x = circleShape->radius;
				rigidBody->velocity.x *= -0.8f;
			}
		}
	}

	// Check all the rigidbodies with the other rigidbodies for collision
	for (std::size_t i = 0; i < m_rigidBodies.size() - 1; i++)
	{
		for (std::size_t j = i + 1; j < m_rigidBodies.size(); j++)
		{
			RigidBody* a = m_rigidBodies[i];
			RigidBody* b = m_rigidBodies[j];

			Contact contact;

			if (CollisionDetection::IsColliding(a, b, contact))
			{
				//contact.ResolveCollision();

				a->isColliding = true;
				b->isColliding = true;
			}
		}
	}
}

void Application::Render()
{
	BeginDrawing();

	Graphics::ClearScreen(DARKGRAY);

	for (const RigidBody* rigidBody : m_rigidBodies)
	{
		if (rigidBody->shape->GetType() == CIRCLE)
		{
			const CircleShape* circleShape = dynamic_cast<CircleShape*>(rigidBody->shape);
			Graphics::DrawFillCircle(rigidBody->position, circleShape->radius, WHITE);
		}
		if (rigidBody->shape->GetType() == BOX)
		{
			const BoxShape* boxShape = dynamic_cast<BoxShape*>(rigidBody->shape);
			Graphics::DrawPolygon(rigidBody->position, boxShape->worldVertices, rigidBody->isColliding ? RED : WHITE);
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
