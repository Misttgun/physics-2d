#include "physics/Shape.h"
#include "physics/Vec2.h"

#include <limits>

CircleShape::CircleShape(const float radius)
{
	m_radius = radius;
}

ShapeType CircleShape::GetType() const
{
	return CIRCLE;
}

std::unique_ptr<Shape> CircleShape::Clone() const
{
	return std::make_unique<CircleShape>(m_radius);
}

void CircleShape::UpdateVertices(const Vec2& position, float angle)
{}

float CircleShape::GetMomentOfInertia() const
{
	return 0.5f * (m_radius * m_radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2>& vertices)
{
	// Initialize the vertices of the polygon shape
    for (auto vertex: vertices) 
	{
        m_localVertices.emplace_back(vertex.x, vertex.y);
        m_worldVertices.emplace_back(vertex.x, vertex.y);
    }
}

ShapeType PolygonShape::GetType() const
{
	return POLYGON;
}

std::unique_ptr<Shape> PolygonShape::Clone() const
{
	return std::make_unique<PolygonShape>(m_localVertices);
}

float PolygonShape::GetMomentOfInertia() const
{
	// TODO: We need to compute the moment of inertia of the polygon correctly!!!
    return 5000;
}

// Translate and rotate local vertices from local to world space
void PolygonShape::UpdateVertices(const Vec2& position, const float angle)
{
	for (std::size_t i = 0; i < m_localVertices.size(); i++)
	{
		// We rotate first and then we do the translation
		m_worldVertices[i] = m_localVertices[i].Rotate(angle);
		m_worldVertices[i] += position;
	}
}

Vec2 PolygonShape::EdgeAt(const std::size_t index) const
{
	const std::size_t currVertex = index;
	const std::size_t nextVertex = (index + 1) % m_worldVertices.size();
	return m_worldVertices[nextVertex] - m_worldVertices[currVertex];
}

float PolygonShape::FindMinimumSeparation(const PolygonShape* other, Vec2& outAxis, Vec2& outPoint) const
{
    float separation = std::numeric_limits<float>::lowest();

    for (std::size_t i = 0; i < m_worldVertices.size(); i++)
    {
        const Vec2 va = m_worldVertices[i];
		const Vec2 edge = EdgeAt(i);
        const Vec2 normal = edge.Perpendicular();

        float minSep = std::numeric_limits<float>::max();
		Vec2 minVertex;

        for (const auto& vb : other->m_worldVertices)
        {
	        const float proj = (vb - va).Dot(normal);

			if(proj < minSep)
			{
				minSep = proj;
				minVertex = vb;
			}
        }
        
		if(minSep > separation)
		{
			separation = minSep;
			outAxis = edge;
			outPoint = minVertex;
		}
    }
    
    return separation;
}

BoxShape::BoxShape(const int width, const int height)
{
	m_width = width;
	m_height = height;

	// Load box local vertices
	m_localVertices.emplace_back( static_cast<float>(-m_width) / 2.0f, static_cast<float>(-m_height) / 2.0f);
	m_localVertices.emplace_back(static_cast<float>(m_width) / 2.0f, static_cast<float>(-m_height) / 2.0f);
	m_localVertices.emplace_back(static_cast<float>(m_width) / 2.0f, static_cast<float>(m_height) / 2.0f);
	m_localVertices.emplace_back(static_cast<float>(-m_width) / 2.0f, static_cast<float>(m_height) / 2.0f);

	m_worldVertices = m_localVertices;
}

ShapeType BoxShape::GetType() const
{
	return BOX;
}

std::unique_ptr<Shape> BoxShape::Clone() const
{
	return std::make_unique<BoxShape>(m_width, m_height);
}

float BoxShape::GetMomentOfInertia() const
{
	return (1 / 12.0f) * static_cast<float>(m_width * m_width + m_height * m_height);
}
