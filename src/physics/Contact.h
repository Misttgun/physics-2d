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