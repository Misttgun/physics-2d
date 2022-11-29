#pragma once

#include "Vec2.h"
#include "RigidBody.h"

struct Contact
{
    RigidBody* a;
    RigidBody* b;

    Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;

    Contact() = default;
    ~Contact() = default;

    void ResolvePenetration();
    void ResolveCollision();
};

inline void Contact::ResolvePenetration()
{
    if (a->IsStatic() && b->IsStatic())
        return;

    float da = (depth / (a->invMass + b->invMass)) * a->invMass;
    float db = (depth / (a->invMass + b->invMass)) * b->invMass;

    a->position -= normal * da;
    b->position += normal * db;
}

inline void Contact::ResolveCollision()
{
    // Apply position correction using the projection method
    ResolvePenetration();

    // Define elasticity (coefficient of restitution e)
    float e = std::min(a->restitution, b->restitution);

    // Calculate the relative velocity between the two objects
    const Vec2 vrel = (a->velocity - b->velocity);

    // Calculate the relative velocity along the normal collision vector
    float vrelDotNormal = vrel.Dot(normal);

    // Now we proceed to calculate the collision impulse
    const Vec2 impulseDirection = normal;
    const float impulseMagnitude = -(1 + e) * vrelDotNormal / (a->invMass + b->invMass);

    const Vec2 impulse = impulseDirection * impulseMagnitude;

    // Apply the impulse vector to both objects in opposite direction
    a->ApplyImpulse(impulse);
    b->ApplyImpulse(-impulse);

}