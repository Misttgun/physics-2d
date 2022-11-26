#include "Application.h"

#include "Graphics.h"
#include "physics/RigidBody.h"
#include "physics/Constants.h"
#include "physics/Force.h"
#include "physics/Shape.h"

bool Application::IsRunning() const
{
	return WindowShouldClose() == false;
}

void Application::Setup()
{
	Graphics::OpenWindow();
	SetTargetFPS(60);

	const CircleShape circleShape(50);
	/*RigidBody* body = new RigidBody(circleShape, Graphics::Width() / 2.0, Graphics::Height() / 2.0, 1.0);
	m_rigidBodies.push_back(body);*/

	const BoxShape boxShape(200, 100);
	RigidBody *body = new RigidBody(boxShape, Graphics::Width() / 2.0, Graphics::Height() / 2.0, 1.0);
	m_rigidBodies.push_back(body);
}

void Application::ProcessInput()
{
	if (IsKeyDown(KEY_UP))
		m_pushForce.y = -50 * PIXELS_PER_METER;
	if (IsKeyDown(KEY_DOWN))
		m_pushForce.y = 50 * PIXELS_PER_METER;
	if (IsKeyDown(KEY_LEFT))
		m_pushForce.x = -50 * PIXELS_PER_METER;
	if (IsKeyDown(KEY_RIGHT))
		m_pushForce.x = 50 * PIXELS_PER_METER;

	if (IsKeyUp(KEY_UP) || IsKeyUp(KEY_DOWN))
		m_pushForce.y = 0;
	if (IsKeyUp(KEY_LEFT) || IsKeyUp(KEY_RIGHT))
		m_pushForce.x = 0;

	if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && m_leftMouseButtonDown == false)
		m_leftMouseButtonDown = true;

	if (IsMouseButtonUp(MOUSE_BUTTON_LEFT) && m_leftMouseButtonDown)
	{
		m_leftMouseButtonDown = false;
		const Vec2 impulse = (m_rigidBodies[0]->position - m_mouseCursor);
		const Vec2 impulse_direction = impulse.UnitVector();
		const float impulse_magnitude = impulse.Magnitude();
		m_rigidBodies[0]->velocity = impulse_direction * impulse_magnitude;
	}

	m_mouseCursor.x = GetMouseX();
	m_mouseCursor.y = GetMouseY();
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

	for (RigidBody *rigidBody : m_rigidBodies)
	{
		// rigidBody->AddForce(m_pushForce);

		rigidBody->AddTorque(200);

		/*const Vec2 gravity = Vec2(0.0f, 9.8f * PIXELS_PER_METER * rigidBody->mass);
		rigidBody->AddForce(gravity);

		const Vec2 drag = Force::GenerateDrag(*rigidBody, 0.01f);
		rigidBody->AddForce(drag);*/

		rigidBody->Update(dt);

		if (rigidBody->shape->GetType() == CIRCLE)
		{
			const CircleShape *circleShape = dynamic_cast<CircleShape *>(rigidBody->shape);

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
}

void Application::Render()
{
	BeginDrawing();

	Graphics::ClearScreen(DARKGRAY);

	for (const RigidBody *rigidBody : m_rigidBodies)
	{
		if (rigidBody->shape->GetType() == CIRCLE)
		{
			const CircleShape *circleShape = dynamic_cast<CircleShape *>(rigidBody->shape);
			Graphics::DrawCircle(rigidBody->position, circleShape->radius, rigidBody->rotation, WHITE);
		}
		if (rigidBody->shape->GetType() == BOX)
		{
			const BoxShape *boxShape = dynamic_cast<BoxShape *>(rigidBody->shape);
			Graphics::DrawPolygon(rigidBody->position, boxShape->worldVertices, WHITE);
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
