#pragma once

#include <memory>
#include <vector>

struct Vec2;

enum ShapeType : uint8_t
{
	CIRCLE,
	POLYGON,
	BOX
};

class Shape
{
public:
	[[nodiscard]] virtual ShapeType GetType() const = 0;
	[[nodiscard]] virtual std::unique_ptr<Shape> Clone() const = 0;
	virtual void UpdateVertices(const Vec2& position, float angle) = 0;
	[[nodiscard]] virtual float GetMomentOfInertia() const = 0;

	Shape() = default;
	virtual ~Shape() = default;
	Shape(const Shape& shape) = delete;
	Shape& operator =(const Shape& shape) = delete;
	Shape(Shape&& shape) = delete;
	Shape& operator = (Shape&& shape) = delete;
};

class CircleShape final : public Shape
{
public:
	float m_radius;

	explicit CircleShape(float radius);
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	void UpdateVertices(const Vec2& position, float angle) override;
	[[nodiscard]] float GetMomentOfInertia() const override;

	CircleShape() = default;
	~CircleShape() override = default;
	CircleShape(const CircleShape& shape) = delete;
	CircleShape& operator =(const CircleShape& shape) = delete;
	CircleShape(CircleShape&& shape) = delete;
	CircleShape& operator = (CircleShape&& shape) = delete;
};

class PolygonShape : public Shape
{
public:
	int m_width;
	int m_height;
	std::vector<Vec2> m_localVertices;
	std::vector<Vec2> m_worldVertices;

	explicit PolygonShape(const std::vector<Vec2>& vertices);
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	[[nodiscard]] float GetMomentOfInertia() const override;
	void UpdateVertices(const Vec2& position, float angle) override;

	[[nodiscard]] Vec2 EdgeAt(std::size_t index) const;
	float FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const;
	[[nodiscard]] int FindIncidentEdge(const Vec2& normal) const;
	static int ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1);

	PolygonShape() = default;
	~PolygonShape() override = default;
	PolygonShape(const PolygonShape& shape) = delete;
	PolygonShape& operator = (const PolygonShape& shape) = delete;
	PolygonShape(PolygonShape&& shape) = delete;
	PolygonShape& operator = (PolygonShape&& shape) = delete;
};

class BoxShape final : public PolygonShape
{
public:
	BoxShape(int width, int height);
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	[[nodiscard]] float GetMomentOfInertia() const override;

	BoxShape() = default;
	~BoxShape() override = default;
	BoxShape(const BoxShape& shape) = delete;
	BoxShape& operator =(const BoxShape& shape) = delete;
	BoxShape(BoxShape&& shape) = delete;
	BoxShape& operator = (BoxShape&& shape) = delete;
};
