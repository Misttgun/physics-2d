#pragma once
#include <vector>

#include "RigidBody.h"
#include "ShapeType.h"

struct Vec2;

struct Shape
{
	virtual ~Shape() = default;
	virtual ShapeType GetType() const = 0;
	virtual Shape* Clone() const = 0;
	virtual void UpdateVertices(const Vec2& position, float angle);
	virtual float GetMomentOfInertia() const = 0;
};

struct CircleShape final : public Shape
{
	float radius;

	CircleShape(float radius_);
	~CircleShape() override;
	ShapeType GetType() const override;
	Shape* Clone() const override;
	void UpdateVertices(const Vec2& position, float angle) override;
	float GetMomentOfInertia() const override;
};

struct PolygonShape : public Shape
{
	std::vector<Vec2> localVertices;
	std::vector<Vec2> worldVertices;

	PolygonShape() = default;
	PolygonShape(const std::vector<Vec2> vertices_);
	~PolygonShape() override;
	ShapeType GetType() const override;
	Shape* Clone() const override;
	float GetMomentOfInertia() const override;
	void UpdateVertices(const Vec2& position, float angle) override;
	Vec2 EdgeAt(int index) const;
	float FindMinimumSeparation(const PolygonShape* other, Vec2& outAxis, Vec2& outPoint) const;
};

struct BoxShape final : public PolygonShape
{
	float width;
	float height;

	BoxShape(float width_, float height_);
	~BoxShape() override;
	ShapeType GetType() const override;
	Shape* Clone() const override;
	float GetMomentOfInertia() const override;
};
