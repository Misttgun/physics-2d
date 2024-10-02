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
	virtual ~Shape() = default;
	[[nodiscard]] virtual ShapeType GetType() const = 0;
	[[nodiscard]] virtual std::unique_ptr<Shape> Clone() const = 0;
	virtual void UpdateVertices(const Vec2& position, float angle) = 0;
	[[nodiscard]] virtual float GetMomentOfInertia() const = 0;

	Shape() = default;
	Shape(const Shape& shape) = default;
	Shape& operator =(const Shape& shape) = default;
	Shape(Shape&& shape) = default;
	Shape& operator = (Shape&& shape) = default;
};

class CircleShape final : public Shape
{
public:
	float m_radius;

	explicit CircleShape(float radius);
	~CircleShape() override = default;
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	void UpdateVertices(const Vec2& position, float angle) override;
	[[nodiscard]] float GetMomentOfInertia() const override;

	CircleShape() = default;
	CircleShape(const CircleShape& shape) = default;
	CircleShape& operator =(const CircleShape& shape) = default;
	CircleShape(CircleShape&& shape) = default;
	CircleShape& operator = (CircleShape&& shape) = default;
};

class PolygonShape : public Shape
{
public:
	std::vector<Vec2> m_localVertices;
	std::vector<Vec2> m_worldVertices;

	explicit PolygonShape(const std::vector<Vec2>& vertices);
	~PolygonShape() override = default;
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	[[nodiscard]] float GetMomentOfInertia() const override;
	void UpdateVertices(const Vec2& position, float angle) override;
	[[nodiscard]] Vec2 EdgeAt(std::size_t index) const;
	float FindMinimumSeparation(const PolygonShape* other, Vec2& outAxis, Vec2& outPoint) const;

	PolygonShape() = default;
	PolygonShape(const PolygonShape& shape) = default;
	PolygonShape& operator = (const PolygonShape& shape) = default;
	PolygonShape(PolygonShape&& shape) = default;
	PolygonShape& operator = (PolygonShape&& shape) = default;
};

class BoxShape final : public PolygonShape
{
public:
	int m_width;
	int m_height;

	BoxShape(int width, int height);
	~BoxShape() override = default;
	[[nodiscard]] ShapeType GetType() const override;
	[[nodiscard]] std::unique_ptr<Shape> Clone() const override;
	[[nodiscard]] float GetMomentOfInertia() const override;

	BoxShape() = default;
	BoxShape(const BoxShape& shape) = default;
	BoxShape& operator =(const BoxShape& shape) = default;
	BoxShape(BoxShape&& shape) = default;
	BoxShape& operator = (BoxShape&& shape) = default;
};
