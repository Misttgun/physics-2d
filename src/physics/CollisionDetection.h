#pragma once

#include "RigidBody.h"
#include "Shape.h"
#include "Contact.h"

struct CollisionDetection
{
    static bool IsColliding(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingCircleCircle(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingPolygonPolygon(RigidBody* a, RigidBody* b, Contact& outContact);
    // static bool IsCollidingPolygonCircle(RigidBody* a, RigidBody* b);
};

inline bool CollisionDetection::IsColliding(RigidBody* a, RigidBody* b, Contact& outContact)
{
    bool aIsCircle = a->shape->GetType() == CIRCLE;
    bool bIsCircle = b->shape->GetType() == CIRCLE;

    bool aIsPolygon = a->shape->GetType() == POLYGON || a->shape->GetType() == BOX; 
    bool bIsPolygon = b->shape->GetType() == POLYGON || b->shape->GetType() == BOX;

    if (aIsCircle && bIsCircle)
        return IsCollidingCircleCircle(a, b, outContact);

    if(aIsPolygon && bIsPolygon)
        return IsCollidingPolygonPolygon(a, b, outContact);
    
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

inline bool CollisionDetection::IsCollidingPolygonPolygon(RigidBody* a, RigidBody* b, Contact& outContact)
{
    const PolygonShape* aPolygonShape = dynamic_cast<PolygonShape*>(a->shape);
    const PolygonShape* bPolygonShape = dynamic_cast<PolygonShape*>(b->shape);

    Vec2 aAxis, bAxis, aPoint, bPoint;

    float abSeparation = aPolygonShape->FindMinimumSeparation(bPolygonShape, aAxis, aPoint);
    float baSeparation = bPolygonShape->FindMinimumSeparation(aPolygonShape, bAxis, bPoint);

    outContact.a = a;
    outContact.b = b;

    if(abSeparation > baSeparation)
    {
        outContact.depth = -abSeparation;
        outContact.normal = aAxis.Normal();
        outContact.start = aPoint;
        outContact.end = aPoint + outContact.normal * outContact.depth; 
    }
    else
    {
        outContact.depth = -baSeparation;
        outContact.normal = -bAxis.Normal();
        outContact.start = bPoint - outContact.normal * outContact.depth;
        outContact.end = bPoint;
    }

    return abSeparation <= 0 && baSeparation <= 0;
}
