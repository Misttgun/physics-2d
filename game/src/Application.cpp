#include "Application.h"
#include "Graphics.h"
#include "raymath.h"
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

	SearchAndSetResourceDir("assets");

	m_resourceManager = std::make_unique<ResourceManager>();

	// Load resources
	m_resourceManager->AddTexture("basketball-image", "basketball.png");
	m_resourceManager->AddTexture("bowlingball-image", "bowlingball.png");
	m_resourceManager->AddTexture("crate-image", "crate.png");
	m_resourceManager->AddTexture("metal-image", "metal.png");
	m_resourceManager->AddTexture("bob-image", "ragdoll/bob.png");
	m_resourceManager->AddTexture("head-image", "ragdoll/head.png");
	m_resourceManager->AddTexture("torso-image", "ragdoll/torso.png");
	m_resourceManager->AddTexture("left-arm-image", "ragdoll/leftArm.png");
	m_resourceManager->AddTexture("right-arm-image", "ragdoll/rightArm.png");
	m_resourceManager->AddTexture("left-leg-image", "ragdoll/leftLeg.png");
	m_resourceManager->AddTexture("right-leg-image", "ragdoll/rightLeg.png");

	m_world = std::make_unique<World>(-9.8f);

	// Add ragdoll parts (rigid bodies)
	const auto bob = std::make_shared<RigidBody>(CircleShape(5), Graphics::Width() / 2.0, Graphics::Height() / 2.0 - 200, 0.0);
	const auto head = std::make_shared<RigidBody>(CircleShape(25), bob->m_position.x, bob->m_position.y + 70, 5.0);
	const auto torso = std::make_shared<RigidBody>(BoxShape(50, 100), head->m_position.x, head->m_position.y + 80, 3.0);
	const auto leftArm = std::make_shared<RigidBody>(BoxShape(15, 70), torso->m_position.x - 32, torso->m_position.y - 10, 1.0);
	const auto rightArm = std::make_shared<RigidBody>(BoxShape(15, 70), torso->m_position.x + 32, torso->m_position.y - 10, 1.0);
	const auto leftLeg = std::make_shared<RigidBody>(BoxShape(20, 90), torso->m_position.x - 20, torso->m_position.y + 97, 1.0);
	const auto rightLeg = std::make_shared<RigidBody>(BoxShape(20, 90), torso->m_position.x + 20, torso->m_position.y + 97, 1.0);

	bob->SetTexture("bob-image");
	head->SetTexture("head-image");
	torso->SetTexture("torso-image");
	leftArm->SetTexture("left-arm-image");
	rightArm->SetTexture("right-arm-image");
	leftLeg->SetTexture("left-leg-image");
	rightLeg->SetTexture("right-leg-image");

	m_world->AddBody(bob);
	m_world->AddBody(head);
	m_world->AddBody(torso);
	m_world->AddBody(leftArm);
	m_world->AddBody(rightArm);
	m_world->AddBody(leftLeg);
	m_world->AddBody(rightLeg);

	// Add joints between ragdoll parts (distance constraints with one anchor point)
	const auto string = std::make_shared<JointConstraint>(bob, head, bob->m_position);
	const auto neck = std::make_shared<JointConstraint>(head, torso, head->m_position + Vec2(0, 25));
	const auto leftShoulder = std::make_shared<JointConstraint>(torso, leftArm, torso->m_position + Vec2(-28, -45));
	const auto rightShoulder = std::make_shared<JointConstraint>(torso, rightArm, torso->m_position + Vec2(+28, -45));
	const auto leftHip = std::make_shared<JointConstraint>(torso, leftLeg, torso->m_position + Vec2(-20, +50));
	const auto rightHip = std::make_shared<JointConstraint>(torso, rightLeg, torso->m_position + Vec2(+20, +50));

	m_world->AddConstraint(string);
	m_world->AddConstraint(neck);
	m_world->AddConstraint(leftShoulder);
	m_world->AddConstraint(rightShoulder);
	m_world->AddConstraint(leftHip);
	m_world->AddConstraint(rightHip);

	/*constexpr int numBodies = 8;
	for (int i = 0; i < numBodies; ++i)
	{
		float mass = i == 0 ? 0.0f : 1.0f;
		const auto body = std::make_shared<RigidBody>(BoxShape(30, 30), Graphics::Width() / 2 - (i * 40), 100, mass);
		body->SetTexture("crate-image");
		m_world->AddBody(body);
	}

	const auto bodies = m_world->GetBodies();
	for (int i = 0; i < numBodies - 1; ++i)
	{
		const auto& a = bodies.at(i);
		const auto& b = bodies.at(i + 1);
		const auto joint = std::make_shared<JointConstraint>(a, b, a->m_position);
		m_world->AddConstraint(joint);
	}*/

	const auto floor = std::make_shared<RigidBody>(BoxShape(Graphics::Width() - 50, 50), Graphics::Width() / 2, Graphics::Height() - 50, 0.0f);
	floor->m_restitution = 0.7f;
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

	/*const auto bigBox = std::make_shared<RigidBody>(BoxShape(200, 200), Graphics::Width() / 2, Graphics::Height() / 2, 0.0f);
	bigBox->m_rotation = 1.4f;
	bigBox->m_restitution = 0.5f;
	bigBox->SetTexture("crate-image");
	m_world->AddBody(bigBox);

	const Vec2 wind(0.5f * PIXELS_PER_METER, 0.0f);
	m_world->AddForce(wind);*/
}

void Application::ProcessInput()
{
	if (IsKeyPressed(KEY_D))
		m_debug = !m_debug;

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
	{
		const auto circle = std::make_shared<RigidBody>(CircleShape(30.0f), GetMouseX(), GetMouseY(), 1.0f);
		circle->m_restitution = 0.7f;
		circle->SetTexture("basketball-image");
		m_world->AddBody(circle);
	}

	if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
	{
		const auto box = std::make_shared<RigidBody>(BoxShape(60, 60), GetMouseX(), GetMouseY(), 1.0f);
		box->m_restitution = 0.2f;
		box->SetTexture("crate-image");
		m_world->AddBody(box);
	}

	if(Vector2Length( GetMouseDelta()) >= 2.0f)
	{
		const Vec2 mouse = Vec2(GetMouseX(), GetMouseY());
		const auto& bob = m_world->GetBodies().at(0);
		const Vec2 direction = (mouse - bob->m_position).Normalize();
		bob->m_position += direction * 2.0f;
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

	const auto bodies = m_world->GetBodies();

	// Draw a line between the bob and the ragdoll head
	const auto& bob = bodies.at(0);
	const auto& head = bodies.at(1);
	Graphics::DrawLine(bob->m_position, head->m_position, {85, 85, 85, 255});

	// Draw all joints anchor points
	if (m_debug)
	{
		for (const auto& constraint : m_world->GetConstraints())
		{
			const Vec2 anchorPoint = constraint->a->LocalToWorld(constraint->aPoint);
			Graphics::DrawFillCircle(anchorPoint, 3.0f, BLUE);
		}
	}

	/*for (const auto& constraint : m_world->GetConstraints())
	{
		const Vec2 pa = constraint->a->LocalToWorld(constraint->aPoint);
		const Vec2 pb = constraint->b->LocalToWorld(constraint->aPoint);
		Graphics::DrawLine(pa, pb, {85, 85, 85, 255});
	}*/

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
