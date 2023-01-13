#pragma once

#include "RigidBody.h"
#include "Shape.h"
#include "Contact.h"
#include <limits>

struct CollisionDetection
{
    static bool IsColliding(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingCircleCircle(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingPolygonPolygon(RigidBody* a, RigidBody* b, Contact& outContact);
    static bool IsCollidingPolygonCircle(RigidBody* polygon, RigidBody* circle, Contact& outContact);
};

inline bool CollisionDetection::IsColliding(RigidBody* a, RigidBody* b, Contact& outContact)
{
    bool aIsCircle = a->shape->GetType() == CIRCLE;
    bool bIsCircle = b->shape->GetType() == CIRCLE;

    bool aIsPolygon = a->shape->GetType() == POLYGON || a->shape->GetType() == BOX;
    bool bIsPolygon = b->shape->GetType() == POLYGON || b->shape->GetType() == BOX;

    if (aIsCircle && bIsCircle)
        return IsCollidingCircleCircle(a, b, outContact);

    if (aIsPolygon && bIsPolygon)
        return IsCollidingPolygonPolygon(a, b, outContact);

    if (aIsPolygon && bIsCircle)
        return IsCollidingPolygonCircle(a, b, outContact);

    if (aIsCircle && bIsPolygon)
        return IsCollidingPolygonCircle(b, a, outContact);

    return false;
}

inline bool CollisionDetection::IsCollidingCircleCircle(RigidBody* a, RigidBody* b, Contact& outContact)
{
    CircleShape* aCircleShape = (CircleShape*)a->shape;
    CircleShape* bCircleShape = (CircleShape*)b->shape;

    const Vec2 ab = b->position - a->position;
    const float radiusSum = aCircleShape->radius + bCircleShape->radius;

    bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

    if (isColliding == false)
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

    if (abSeparation > baSeparation)
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

inline bool CollisionDetection::IsCollidingPolygonCircle(RigidBody* polygon, RigidBody* circle, Contact& outContact)
{
    const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(polygon->shape);
    const CircleShape* circleShape = dynamic_cast<CircleShape*>(circle->shape);
    const std::vector<Vec2>& polygonVertices = polygonShape->worldVertices;

    bool isOutside = false;
    Vec2 minCurrVertex;
    Vec2 minNextVertex;
    float distanceCircleEdge = std::numeric_limits<float>::lowest();

    for (std::size_t i = 0; i < polygonVertices.size(); i++)
    {
        int currVertex = i;
        int nextVertex = (i + 1) % polygonVertices.size();
        Vec2 edge = polygonShape->EdgeAt(currVertex);
        Vec2 normal = edge.Normal();

        // Compare the circle cente with the polygon vertex
        Vec2 circleCenter = circle->position - polygonVertices[currVertex];
        float projection = circleCenter.Dot(normal);

        // If we found a dot product projection that is in the positive side of the normal
        if (projection > 0)
        {
            // Circle center is outside the polygon
            distanceCircleEdge = projection;
            minCurrVertex = polygonShape->worldVertices[currVertex];
            minNextVertex = polygonShape->worldVertices[nextVertex];
            isOutside = true;
            break;
        }
        else
        {
            // Circle center is inside the polygon, find the min edge (the one with the least negative projection)
            if (projection > distanceCircleEdge)
            {
                distanceCircleEdge = projection;
                minCurrVertex = polygonVertices[currVertex];
                minNextVertex = polygonVertices[nextVertex];
            }
        }
    }

    if (isOutside)
    {
        ///////////////////////////////////////
        // Check if we are inside region A:
        ///////////////////////////////////////
        Vec2 v1 = circle->position - minCurrVertex; // vector from the nearest vertex to the circle center
        Vec2 v2 = minNextVertex - minCurrVertex; // the nearest edge (from curr vertex to next vertex)
        if (v1.Dot(v2) < 0)
        {
            // Distance from vertex to circle center is greater than radius... no collision
            if (v1.Magnitude() > circleShape->radius)
                return false;

            // Detected collision in region A:
            outContact.a = polygon;
            outContact.b = circle;
            outContact.depth = circleShape->radius - v1.Magnitude();
            outContact.normal = v1.Normalize();
            outContact.start = circle->position + (outContact.normal * -circleShape->radius);
            outContact.end = outContact.start + (outContact.normal * outContact.depth);
        }
        else
        {
            ///////////////////////////////////////
            // Check if we are inside region B:
            ///////////////////////////////////////
            v1 = circle->position - minNextVertex; // vector from the next nearest vertex to the circle center
            v2 = minCurrVertex - minNextVertex;   // the nearest edge
            if (v1.Dot(v2) < 0)
            {
                // Distance from vertex to circle center is greater than radius... no collision
                if (v1.Magnitude() > circleShape->radius)
                    return false;

                // Detected collision in region B:
                outContact.a = polygon;
                outContact.b = circle;
                outContact.depth = circleShape->radius - v1.Magnitude();
                outContact.normal = v1.Normalize();
                outContact.start = circle->position + (outContact.normal * -circleShape->radius);
                outContact.end = outContact.start + (outContact.normal * outContact.depth);
            }
            else
            {
                ///////////////////////////////////////
                // We are inside region C:
                ///////////////////////////////////////
                if (distanceCircleEdge > circleShape->radius)
                    // No collision... Distance between the closest distance and the circle center is greater than the radius.
                    return false;

                // Detected collision in region C:
                outContact.a = polygon;
                outContact.b = circle;
                outContact.depth = circleShape->radius - distanceCircleEdge;
                outContact.normal = (minNextVertex - minCurrVertex).Normal();
                outContact.start = circle->position - (outContact.normal * circleShape->radius);
                outContact.end = outContact.start + (outContact.normal * outContact.depth);

            }
        }
    }
    else
    {
        // The center of circle is inside the polygon... it is definitely colliding!
        outContact.a = polygon;
        outContact.b = circle;
        outContact.depth = circleShape->radius - distanceCircleEdge;
        outContact.normal = (minNextVertex - minCurrVertex).Normal();
        outContact.start = circle->position - (outContact.normal * circleShape->radius);
        outContact.end = outContact.start + (outContact.normal * outContact.depth);
    }

    return true;
}
