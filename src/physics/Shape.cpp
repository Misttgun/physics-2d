#include "Shape.h"
#include "Vec2.h"

#include <limits>

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

void CircleShape::UpdateVertices(const Vec2& position, float angle)
{
	return;
}

float CircleShape::GetMomentOfInertia() const
{
	return 0.5f * (radius * radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2> vertices_)
{
	// Initialize the vertices of the polygon shape
    for (auto vertex: vertices_) 
	{
        localVertices.push_back(vertex);
        worldVertices.push_back(vertex);
    }
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
	// TODO: We need to compute the moment of inertia of the polygon correctly!!!
    return 5000;
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

Vec2 PolygonShape::EdgeAt(int index) const
{
	int currVertex = index;
	int nextVertex = (index + 1) % worldVertices.size();
	return worldVertices[nextVertex] - worldVertices[currVertex];
}

float PolygonShape::FindMinimumSeparation(const PolygonShape* other, Vec2& outAxis, Vec2& outPoint) const
{
    float seperation = std::numeric_limits<float>::lowest();

    for (std::size_t i = 0; i < worldVertices.size(); i++)
    {
        const Vec2 va = worldVertices[i];
		const Vec2 edge = EdgeAt(i);
        const Vec2 normal = edge.Normal();

        float minSep = std::numeric_limits<float>::max();
		Vec2 minVertex;

        for (std::size_t j = 0; j < other->worldVertices.size(); j++)
        {
            const Vec2 vb = other->worldVertices[j];
            float proj = (vb - va).Dot(normal);

			if(proj < minSep)
			{
				minSep = proj;
				minVertex = vb;
			}
        }
        
		if(minSep > seperation)
		{
			seperation = minSep;
			outAxis = edge;
			outPoint = minVertex;
		}
    }
    
    return seperation;
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
