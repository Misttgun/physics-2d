#pragma once

#include "RigidBody.h"
#include "Shape.h"
#include "Contact.h"

struct CollisionDetection
{
    static bool IsColliding(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingCircleCircle(RigidBody* a, RigidBody* b, Contact& outContact);
    // static bool IsCollidingPolygonPolygon(RigidBody* a, RigidBody* b);
    // static bool IsCollidingPolygonCircle(RigidBody* a, RigidBody* b);
};

inline bool CollisionDetection::IsColliding(RigidBody* a, RigidBody* b, Contact& outContact)
{
    bool aIsCircle = a->shape->GetType() == CIRCLE;
    bool bIsCircle = b->shape->GetType() == CIRCLE;

    if (aIsCircle && bIsCircle)
        return IsCollidingCircleCircle(a, b, outContact);

    return false;
}

inline bool CollisionDetection::IsCollidingCircleCircle(RigidBody* a, RigidBody* b, Contact& outContact)
{
    CircleShape* aCircleShape = (CircleShape*)a->shape;
    CircleShape* bCircleShape = (CircleShape*)b->shape;

    const Vec2 ab = b->position - a->position;
    const float radiusSum = aCircleShape->radius + bCircleShape->radius;

    bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

    if(isColliding == false)
        return false;

    outContact.a = a;
    outContact.b = b;
    outContact.normal = ab.Normalized();
    outContact.start = b->position - outContact.normal * bCircleShape->radius;
    outContact.end = a->position + outContact.normal * aCircleShape->radius;
    outContact.depth = (outContact.end - outContact.start).Magnitude();

    return true;
}
