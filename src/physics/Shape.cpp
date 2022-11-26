#include "Shape.h"
#include "Vec2.h"

CircleShape::CircleShape(const float radius_)
{
	radius = radius_;
}

CircleShape::~CircleShape() = default;

ShapeType CircleShape::GetType() const
{
	return CIRCLE;
}

Shape* CircleShape::Clone() const
{
	return new CircleShape(radius);
}

float CircleShape::GetMomentOfInertia() const
{
	return 0.5f * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices_)
{
}

PolygonShape::~PolygonShape() = default;

ShapeType PolygonShape::GetType() const
{
	return POLYGON;
}

Shape* PolygonShape::Clone() const
{
	return new PolygonShape(localVertices);
}

float PolygonShape::GetMomentOfInertia() const
{
	return 0.0f;
}

// Translate and rotate local vertices from local to world space
void PolygonShape::UpdateVertices(const Vec2& position, const float angle)
{
	for (std::size_t i = 0; i < localVertices.size(); i++)
	{
		// We rotate first and then we do the translation
		worldVertices[i] = localVertices[i].Rotate(angle);
		worldVertices[i] += position;
	}
}

BoxShape::BoxShape(const float width_, const float height_)
{
	width = width_;
	height = height_;

	// Load box local vertices
	localVertices.emplace_back(Vec2(-width / 2, -height / 2));
	localVertices.emplace_back(Vec2(width / 2, -height / 2));
	localVertices.emplace_back(Vec2(width / 2, height / 2));
	localVertices.emplace_back(Vec2(-width / 2, height / 2));

	worldVertices = localVertices;
}

BoxShape::~BoxShape() = default;

ShapeType BoxShape::GetType() const
{
	return BOX;
}

Shape* BoxShape::Clone() const
{
	return new BoxShape(width, height);
}

float BoxShape::GetMomentOfInertia() const
{
	return (1 / 12.0f) * (width * width + height * height);
}
