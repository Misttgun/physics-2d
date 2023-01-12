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

    // Define elasticity (coefficient of restitution e) and friction
    float e = std::min(a->restitution, b->restitution);
    float f = std::min(a->friction, b->friction);

    // Calculate the relative velocity between the two objects
    const Vec2 ra = end - a->position;
    const Vec2 rb = start - b->position;
    const Vec2 va = a->velocity + Vec2(-a->angularVelocity * ra.y, a->angularVelocity * ra.x);
    const Vec2 vb = b->velocity + Vec2(-b->angularVelocity * rb.y, b->angularVelocity * rb.x);
    const Vec2 vrel = va - vb;

    // Now we proceed to calculate the collision impulse along the normal
    float vrelDotNormal = vrel.Dot(normal);
    const Vec2 impulseDirectionN = normal;
    const float impulseMagnitudeN = -(1 + e) * vrelDotNormal / ((a->invMass + b->invMass) + ra.Cross(normal) * ra.Cross(normal) * a->invInertia + rb.Cross(normal) * rb.Cross(normal) * b->invInertia);
    const Vec2 impulseN = impulseDirectionN * impulseMagnitudeN;

    // Now we proceed to calculate the collision impulse along the tangent
    const Vec2 tangent = normal.Normal();
    const float vrelDotTangent = vrel.Dot(tangent);
    const Vec2 impulseDirectionT = tangent;
    const float impulseMagnitudeT = f * -(1 + e) * vrelDotTangent / ((a->invMass + b->invMass) + ra.Cross(tangent) * ra.Cross(tangent) * a->invInertia + rb.Cross(tangent) * rb.Cross(tangent) * b->invInertia);
    const Vec2 impulseT = impulseDirectionT * impulseMagnitudeT;

    // Calculate the final impulse combining normal and tangent impulses
    const Vec2 impulse = impulseN + impulseT;

    // Apply the impulse vector to both objects in opposite direction
    a->ApplyImpulse(impulse, ra);
    b->ApplyImpulse(-impulse, rb);

}