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

void CircleShape::UpdateVertices(const Vec2& position, float angle) {}

float CircleShape::GetMomentOfInertia() const
{
	return 0.5f * (m_radius * m_radius);
}

PolygonShape::PolygonShape(const std::vector<Vec2>& vertices)
{
	float minX = std::numeric_limits<float>::max();
	float minY = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::lowest();
	float maxY = std::numeric_limits<float>::lowest();

	// Initialize the vertices of the polygon shape
	for (auto vertex : vertices)
	{
		m_localVertices.emplace_back(vertex.x, vertex.y);
		m_worldVertices.emplace_back(vertex.x, vertex.y);

		// Find min and max X and Y to calculate polygon width and height
		minX = std::min(minX, vertex.x);
		maxX = std::max(maxX, vertex.x);
		minY = std::min(minY, vertex.y);
		maxY = std::max(maxY, vertex.y);
	}

	m_width = static_cast<int>(maxX - minX);
	m_height = static_cast<int>(maxY - minY);
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
	float acc0 = 0;
	float acc1 = 0;
	for (size_t i = 0; i < m_localVertices.size(); i++)
	{
		const Vec2 a = m_localVertices[i];
		const Vec2 b = m_localVertices[(i + 1) % m_localVertices.size()];
		const float cross = abs(a.Cross(b));
		acc0 += cross * (a.Dot(a) + b.Dot(b) + a.Dot(b));
		acc1 += cross;
	}

	return acc0 / 6 / acc1;
}

// Translate and rotate local vertices from local to world space
void PolygonShape::UpdateVertices(const Vec2& position, const float angle)
{
	for (size_t i = 0; i < m_localVertices.size(); i++)
	{
		// We rotate first and then we do the translation
		m_worldVertices[i] = m_localVertices[i].Rotate(angle);
		m_worldVertices[i] += position;
	}
}

Vec2 PolygonShape::EdgeAt(const size_t index) const
{
	const size_t currVertex = index;
	const size_t nextVertex = (index + 1) % m_worldVertices.size();
	return m_worldVertices[nextVertex] - m_worldVertices[currVertex];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const
{
	float separation = std::numeric_limits<float>::lowest();

	// Loop all the vertices of "this" polygon
	for (size_t i = 0; i < this->m_worldVertices.size(); i++)
	{
		Vec2 va = this->m_worldVertices[i];
		Vec2 normal = this->EdgeAt(i).Perpendicular();

		// Loop all the vertices of the "other" polygon
		float minSep = std::numeric_limits<float>::max();
		Vec2 minVertex;

		for (const auto& vb : other->m_worldVertices)
		{
			const float proj = (vb - va).Dot(normal);
			if (proj < minSep)
			{
				minSep = proj;
				minVertex = vb;
			}
		}

		if (minSep > separation)
		{
			separation = minSep;
			indexReferenceEdge = static_cast<int>(i);
			supportPoint = minVertex;
		}
	}

	return separation;
}

int PolygonShape::FindIncidentEdge(const Vec2& normal) const
{
	int indexIncidentEdge = 0;
	float minProj = std::numeric_limits<float>::max();

	for (size_t i = 0; i < this->m_worldVertices.size(); ++i)
	{
		const Vec2 edgeNormal = this->EdgeAt(i).Perpendicular();
		const float proj = edgeNormal.Dot(normal);
		if (proj < minProj)
		{
			minProj = proj;
			indexIncidentEdge = static_cast<int>(i);
		}
	}

	return indexIncidentEdge;
}

int PolygonShape::ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	const Vec2 normal = (c1 - c0).Normalize();
	const float dist0 = (contactsIn[0] - c0).Cross(normal);
	const float dist1 = (contactsIn[1] - c0).Cross(normal);

	// If the points are behind the plane
	if (dist0 <= 0)
		contactsOut[numOut++] = contactsIn[0];

	if (dist1 <= 0)
		contactsOut[numOut++] = contactsIn[1];

	// If the points are on different sides of the plane (one distance is negative and the other is positive)
	if (dist0 * dist1 < 0)
	{
		const float totalDist = dist0 - dist1;

		// Find the intersection using linear interpolation: lerp(start,end) => start + t*(end-start)
		const float t = dist0 / (totalDist);
		const Vec2 contact = contactsIn[0] + (contactsIn[1] - contactsIn[0]) * t;
		contactsOut[numOut] = contact;
		numOut++;
	}

	return numOut;
}

BoxShape::BoxShape(const int width, const int height)
{
	m_width = width;
	m_height = height;

	const auto fWidth = static_cast<float>(m_width);
	const auto fHeight = static_cast<float>(m_height);

	// Load box local vertices
	m_localVertices.emplace_back(-fWidth / 2.0f, -fHeight / 2.0f);
	m_localVertices.emplace_back(+fWidth / 2.0f, -fHeight / 2.0f);
	m_localVertices.emplace_back(+fWidth / 2.0f, +fHeight / 2.0f);
	m_localVertices.emplace_back(-fWidth / 2.0f, +fHeight / 2.0f);

	m_worldVertices.emplace_back(-fWidth / 2.0f, -fHeight / 2.0f);
	m_worldVertices.emplace_back(+fWidth / 2.0f, -fHeight / 2.0f);
	m_worldVertices.emplace_back(+fWidth / 2.0f, +fHeight / 2.0f);
	m_worldVertices.emplace_back(-fWidth / 2.0f, +fHeight / 2.0f);
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
