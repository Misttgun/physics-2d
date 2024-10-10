#pragma once

#include "RigidBody.h"
#include "physics/Shape.h"

struct Contact
{
	std::shared_ptr<RigidBody> a;
	std::shared_ptr<RigidBody> b;

	Vec2 start;
	Vec2 end;

	Vec2 normal;
	float depth;
};

inline bool IsCollidingCircleCircle(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, std::vector<Contact>& outContacts)
{
	const CircleShape* aCircleShape = dynamic_cast<CircleShape*>(a->m_shape.get());
	const CircleShape* bCircleShape = dynamic_cast<CircleShape*>(b->m_shape.get());

	const Vec2 ab = b->m_position - a->m_position;
	const float radiusSum = aCircleShape->m_radius + bCircleShape->m_radius;

	const bool isColliding = ab.MagnitudeSquared() <= (radiusSum * radiusSum);

	if (isColliding == false)
		return false;

	Contact contact;
	contact.a = a;
	contact.b = b;
	contact.normal = ab.Normalized();
	contact.start = b->m_position - contact.normal * bCircleShape->m_radius;
	contact.end = a->m_position + contact.normal * aCircleShape->m_radius;
	contact.depth = (contact.end - contact.start).Magnitude();

	outContacts.push_back(contact);

	return true;
}

inline bool IsCollidingPolygonPolygon(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, std::vector<Contact>& outContacts)
{
	const PolygonShape* aPolygonShape = dynamic_cast<PolygonShape*>(a->m_shape.get());
	const PolygonShape* bPolygonShape = dynamic_cast<PolygonShape*>(b->m_shape.get());

	int aIndexReferenceEdge, bIndexReferenceEdge;
	Vec2 aSupportPoint, bSupportPoint;

	const float abSeparation = aPolygonShape->FindMinSeparation(bPolygonShape, aIndexReferenceEdge, aSupportPoint);
	if (abSeparation >= 0)
		return false;

	const float baSeparation = bPolygonShape->FindMinSeparation(aPolygonShape, bIndexReferenceEdge, bSupportPoint);
	if (baSeparation >= 0)
		return false;

	const PolygonShape* referenceShape;
	const PolygonShape* incidentShape;
	size_t indexReferenceEdge;

	if (abSeparation > baSeparation)
	{
		referenceShape = aPolygonShape;
		incidentShape = bPolygonShape;
		indexReferenceEdge = aIndexReferenceEdge;
	}
	else
	{
		referenceShape = bPolygonShape;
		incidentShape = aPolygonShape;
		indexReferenceEdge = bIndexReferenceEdge;
	}

	// Find the reference edge based on the index that returned from the function
	const Vec2 referenceEdge = referenceShape->EdgeAt(indexReferenceEdge);

	// Clipping 
	const auto incidentIndex = incidentShape->FindIncidentEdge(referenceEdge.Perpendicular());
	const auto incidentNextIndex = (incidentIndex + 1) % incidentShape->m_worldVertices.size();
	const Vec2 v0 = incidentShape->m_worldVertices[incidentIndex];
	const Vec2 v1 = incidentShape->m_worldVertices[incidentNextIndex];

	std::vector<Vec2> contactPoints = {v0, v1};
	std::vector<Vec2> clippedPoints = contactPoints;
	for (size_t i = 0; i < referenceShape->m_worldVertices.size(); i++)
	{
		if (i == indexReferenceEdge)
			continue;

		Vec2 c0 = referenceShape->m_worldVertices[i];
		Vec2 c1 = referenceShape->m_worldVertices[(i + 1) % referenceShape->m_worldVertices.size()];

		const int numClipped = PolygonShape::ClipSegmentToLine(contactPoints, clippedPoints, c0, c1);
		if (numClipped < 2)
			break;

		contactPoints = clippedPoints; // Make the next contact points the ones that were just clipped
	}

	const Vec2 vref = referenceShape->m_worldVertices[indexReferenceEdge];

	// Loop all clipped points, but only consider those where separation is negative (objects are penetrating each other)
	for (const auto& vclip : clippedPoints)
	{
		const float separation = (vclip - vref).Dot(referenceEdge.Perpendicular());
		if (separation <= 0)
		{
			Contact contact;
			contact.a = a;
			contact.b = b;
			contact.normal = referenceEdge.Perpendicular();
			contact.start = vclip;
			contact.end = vclip + contact.normal * -separation;

			if (baSeparation >= abSeparation)
			{
				std::swap(contact.start, contact.end); // The start-end points are always from "a" to "b"
				contact.normal *= -1.0; // The collision normal is always from "a" to "b"
			}

			outContacts.push_back(contact);
		}
	}
	return true;
}

inline bool IsCollidingPolygonCircle(const std::shared_ptr<RigidBody>& polygon, const std::shared_ptr<RigidBody>& circle, std::vector<Contact>& outContacts)
{
	const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(polygon->m_shape.get());
	const CircleShape* circleShape = dynamic_cast<CircleShape*>(circle->m_shape.get());
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
		Vec2 circleCenter = circle->m_position - polygonVertices[currVertex];
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

	Contact contact;
	contact.a = polygon;
	contact.b = circle;

	if (isOutside)
	{
		// Check if we are inside region A:
		Vec2 v1 = circle->m_position - minCurrVertex; // vector from the nearest vertex to the circle center
		Vec2 v2 = minNextVertex - minCurrVertex; // the nearest edge (from curr vertex to next vertex)
		if (v1.Dot(v2) < 0)
		{
			// Distance from vertex to circle center is greater than radius... no collision
			const float magnitude = v1.Magnitude();
			if (magnitude > circleShape->m_radius)
				return false;

			// Detected collision in region A:
			contact.depth = circleShape->m_radius - magnitude;
			contact.normal = v1.Normalize();
			contact.start = circle->m_position + (contact.normal * -circleShape->m_radius);
			contact.end = contact.start + (contact.normal * contact.depth);
		}
		else
		{
			// Check if we are inside region B:
			v1 = circle->m_position - minNextVertex; // vector from the next nearest vertex to the circle center
			v2 = minCurrVertex - minNextVertex; // the nearest edge
			if (v1.Dot(v2) < 0)
			{
				// Distance from vertex to circle center is greater than radius... no collision
				const float magnitude = v1.Magnitude();
				if (magnitude > circleShape->m_radius)
					return false;

				// Detected collision in region B:
				contact.depth = circleShape->m_radius - magnitude;
				contact.normal = v1.Normalize();
				contact.start = circle->m_position + (contact.normal * -circleShape->m_radius);
				contact.end = contact.start + (contact.normal * contact.depth);
			}
			else
			{
				// We are inside region C:
				if (distanceCircleEdge > circleShape->m_radius)
					// No collision... Distance between the closest distance and the circle center is greater than the radius.
					return false;

				// Detected collision in region C:
				contact.depth = circleShape->m_radius - distanceCircleEdge;
				contact.normal = (minNextVertex - minCurrVertex).Perpendicular();
				contact.start = circle->m_position - (contact.normal * circleShape->m_radius);
				contact.end = contact.start + (contact.normal * contact.depth);
			}
		}
	}
	else
	{
		// The center of circle is inside the polygon... it is definitely colliding!
		contact.depth = circleShape->m_radius - distanceCircleEdge;
		contact.normal = (minNextVertex - minCurrVertex).Perpendicular();
		contact.start = circle->m_position - (contact.normal * circleShape->m_radius);
		contact.end = contact.start + (contact.normal * contact.depth);
	}

	outContacts.push_back(contact);

	return true;
}

inline bool IsColliding(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, std::vector<Contact>& outContacts)
{
	const bool aIsCircle = a->m_shape->GetType() == CIRCLE;
	const bool bIsCircle = b->m_shape->GetType() == CIRCLE;

	const bool aIsPolygon = a->m_shape->GetType() == POLYGON || a->m_shape->GetType() == BOX;
	const bool bIsPolygon = b->m_shape->GetType() == POLYGON || b->m_shape->GetType() == BOX;

	if (aIsCircle && bIsCircle)
		return IsCollidingCircleCircle(a, b, outContacts);

	if (aIsPolygon && bIsPolygon)
		return IsCollidingPolygonPolygon(a, b, outContacts);

	if (aIsPolygon && bIsCircle)
		return IsCollidingPolygonCircle(a, b, outContacts);

	if (aIsCircle && bIsPolygon)
		return IsCollidingPolygonCircle(b, a, outContacts);

	return false;
}
