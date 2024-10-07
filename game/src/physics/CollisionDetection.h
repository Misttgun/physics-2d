#pragma once

#include "RigidBody.h"
#include "physics/Shape.h"

struct Contact
{
	RigidBody* a;
	RigidBody* b;

	Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;
};

inline bool IsCollidingCircleCircle(RigidBody& a, RigidBody& b, Contact& outContact)
{
	const CircleShape* aCircleShape = dynamic_cast<CircleShape*>(a.m_shape.get());
	const CircleShape* bCircleShape = dynamic_cast<CircleShape*>(b.m_shape.get());

	const Vec2 ab = b.m_position - a.m_position;
	const float radiusSum = aCircleShape->m_radius + bCircleShape->m_radius;

	const bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

	if (isColliding == false)
		return false;

	outContact.a = &a;
	outContact.b = &b;
	outContact.normal = ab.Normalized();
	outContact.start = b.m_position - outContact.normal * bCircleShape->m_radius;
	outContact.end = a.m_position + outContact.normal * aCircleShape->m_radius;
	outContact.depth = (outContact.end - outContact.start).Magnitude();

	return true;
}

inline bool IsCollidingPolygonPolygon(RigidBody& a, RigidBody& b, Contact& outContact)
{
	const PolygonShape* aPolygonShape = dynamic_cast<PolygonShape*>(a.m_shape.get());
	const PolygonShape* bPolygonShape = dynamic_cast<PolygonShape*>(b.m_shape.get());

	Vec2 aAxis, bAxis, aPoint, bPoint;

	const float abSeparation = aPolygonShape->FindMinimumSeparation(bPolygonShape, aAxis, aPoint);
	const float baSeparation = bPolygonShape->FindMinimumSeparation(aPolygonShape, bAxis, bPoint);

	outContact.a = &a;
	outContact.b = &b;

	if (abSeparation > baSeparation)
	{
		outContact.depth = -abSeparation;
		outContact.normal = aAxis.Perpendicular();
		outContact.start = aPoint;
		outContact.end = aPoint + outContact.normal * outContact.depth;
	}
	else
	{
		outContact.depth = -baSeparation;
		outContact.normal = -bAxis.Perpendicular();
		outContact.start = bPoint - outContact.normal * outContact.depth;
		outContact.end = bPoint;
	}

	return abSeparation <= 0 && baSeparation <= 0;
}

inline bool IsCollidingPolygonCircle(RigidBody& polygon, RigidBody& circle, Contact& outContact)
{
	const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(polygon.m_shape.get());
	const CircleShape* circleShape = dynamic_cast<CircleShape*>(circle.m_shape.get());
	const std::vector<Vec2>& polygonVertices = polygonShape->m_worldVertices;

	bool isOutside = false;
	Vec2 minCurrVertex;
	Vec2 minNextVertex;
	float distanceCircleEdge = std::numeric_limits<float>::lowest();

	for (std::size_t i = 0; i < polygonVertices.size(); i++)
	{
		const auto currVertex = i;
		const auto nextVertex = (i + 1) % polygonVertices.size();
		Vec2 edge = polygonShape->EdgeAt(currVertex);
		Vec2 normal = edge.Perpendicular();

		// Compare the circle center with the polygon vertex
		Vec2 circleCenter = circle.m_position - polygonVertices[currVertex];
		const float projection = circleCenter.Dot(normal);

		// If we found a dot product projection that is in the positive side of the normal
		if (projection > 0)
		{
			// Circle center is outside the polygon
			distanceCircleEdge = projection;
			minCurrVertex = polygonShape->m_worldVertices[currVertex];
			minNextVertex = polygonShape->m_worldVertices[nextVertex];
			isOutside = true;
			break;
		}

		// Circle center is inside the polygon, find the min edge (the one with the least negative projection)
		if (projection > distanceCircleEdge)
		{
			distanceCircleEdge = projection;
			minCurrVertex = polygonVertices[currVertex];
			minNextVertex = polygonVertices[nextVertex];
		}
	}

	outContact.a = &polygon;
	outContact.b = &circle;

	if (isOutside)
	{
		///////////////////////////////////////
		// Check if we are inside region A:
		///////////////////////////////////////
		Vec2 v1 = circle.m_position - minCurrVertex; // vector from the nearest vertex to the circle center
		Vec2 v2 = minNextVertex - minCurrVertex; // the nearest edge (from curr vertex to next vertex)
		if (v1.Dot(v2) < 0)
		{
			// Distance from vertex to circle center is greater than radius... no collision
			const float magnitude = v1.Magnitude();
			if (magnitude > circleShape->m_radius)
				return false;

			// Detected collision in region A:
			outContact.depth = circleShape->m_radius - magnitude;
			outContact.normal = v1.Normalize();
			outContact.start = circle.m_position + (outContact.normal * -circleShape->m_radius);
			outContact.end = outContact.start + (outContact.normal * outContact.depth);
		}
		else
		{
			///////////////////////////////////////
			// Check if we are inside region B:
			///////////////////////////////////////
			v1 = circle.m_position - minNextVertex; // vector from the next nearest vertex to the circle center
			v2 = minCurrVertex - minNextVertex; // the nearest edge
			if (v1.Dot(v2) < 0)
			{
				// Distance from vertex to circle center is greater than radius... no collision
				const float magnitude = v1.Magnitude();
				if (magnitude > circleShape->m_radius)
					return false;

				// Detected collision in region B:
				outContact.depth = circleShape->m_radius - magnitude;
				outContact.normal = v1.Normalize();
				outContact.start = circle.m_position + (outContact.normal * -circleShape->m_radius);
				outContact.end = outContact.start + (outContact.normal * outContact.depth);
			}
			else
			{
				///////////////////////////////////////
				// We are inside region C:
				///////////////////////////////////////
				if (distanceCircleEdge > circleShape->m_radius)
					// No collision... Distance between the closest distance and the circle center is greater than the radius.
					return false;

				// Detected collision in region C:
				outContact.depth = circleShape->m_radius - distanceCircleEdge;
				outContact.normal = (minNextVertex - minCurrVertex).Perpendicular();
				outContact.start = circle.m_position - (outContact.normal * circleShape->m_radius);
				outContact.end = outContact.start + (outContact.normal * outContact.depth);
			}
		}
	}
	else
	{
		// The center of circle is inside the polygon... it is definitely colliding!
		outContact.depth = circleShape->m_radius - distanceCircleEdge;
		outContact.normal = (minNextVertex - minCurrVertex).Perpendicular();
		outContact.start = circle.m_position - (outContact.normal * circleShape->m_radius);
		outContact.end = outContact.start + (outContact.normal * outContact.depth);
	}

	return true;
}

inline bool IsColliding(RigidBody& a, RigidBody& b, Contact& outContact)
{
	const bool aIsCircle = a.m_shape->GetType() == CIRCLE;
	const bool bIsCircle = b.m_shape->GetType() == CIRCLE;

	const bool aIsPolygon = a.m_shape->GetType() == POLYGON || a.m_shape->GetType() == BOX;
	const bool bIsPolygon = b.m_shape->GetType() == POLYGON || b.m_shape->GetType() == BOX;

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


inline void ResolvePenetration(const Contact& contact)
{
	const auto a = contact.a;
	const auto b = contact.b;

	if (a->IsStatic() && b->IsStatic())
		return;

	const float da = (contact.depth / (a->m_invMass + b->m_invMass)) * a->m_invMass;
	const float db = (contact.depth / (a->m_invMass + b->m_invMass)) * b->m_invMass;

	a->m_position -= contact.normal * da * 0.8f;
	b->m_position += contact.normal * db * 0.8f;

	a->m_shape->UpdateVertices(a->m_position, a->m_rotation);
	b->m_shape->UpdateVertices(b->m_position, b->m_rotation);
}

inline void ResolveCollision(const Contact& contact)
{
	const auto a = contact.a;
	const auto b = contact.b;

	// Apply position correction using the projection method
	ResolvePenetration(contact);

	// Define elasticity (coefficient of restitution e) and friction
	const float e = std::min(a->m_restitution, b->m_restitution);
	const float f = std::min(a->m_friction, b->m_friction);

	// Calculate the relative velocity between the two objects
	const Vec2 ra = contact.end - a->m_position;
	const Vec2 rb = contact.start - b->m_position;
	const Vec2 va = a->m_velocity + Vec2(-a->m_angularVelocity * ra.y, a->m_angularVelocity * ra.x);
	const Vec2 vb = b->m_velocity + Vec2(-b->m_angularVelocity * rb.y, b->m_angularVelocity * rb.x);
	const Vec2 vRel = va - vb;

	// Now we proceed to calculate the collision impulse along the normal
	const Vec2 normal = contact.normal;
	const float vRelDotNormal = vRel.Dot(normal);
	const float impulseMagnitudeN = -(1 + e) * vRelDotNormal / 
		((a->m_invMass + b->m_invMass) + ra.Cross(normal) * ra.Cross(normal) * a->m_invInertia + rb.Cross(normal) * rb.Cross(normal) * b->m_invInertia);
	const Vec2 impulseN = normal * impulseMagnitudeN;

	// Now we proceed to calculate the collision impulse along the tangent
	const Vec2 tangent = contact.normal.Perpendicular();
	const float vRelDotTangent = vRel.Dot(tangent);
	const float impulseMagnitudeT = f * -(1 + e) * vRelDotTangent / 
		((a->m_invMass + b->m_invMass) + ra.Cross(tangent) * ra.Cross(tangent) * a->m_invInertia + rb.Cross(tangent) * rb.Cross(tangent) * b->m_invInertia);
	const Vec2 impulseT = tangent * impulseMagnitudeT;

	// Calculate the final impulse combining normal and tangent impulses
	const Vec2 impulse = impulseN + impulseT;

	// Apply the impulse vector to both objects in opposite direction
	a->ApplyImpulse(impulse, ra);
	b->ApplyImpulse(-impulse, rb);
}