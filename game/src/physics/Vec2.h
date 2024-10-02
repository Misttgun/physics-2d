#pragma once

struct Vec2
{
	float x;
	float y;

	Vec2();
	Vec2(float x, float y);

	[[nodiscard]] Vec2 Rotate(float angle) const;
	[[nodiscard]] float Magnitude() const;
	[[nodiscard]] float MagnitudeSquared() const;

	Vec2& Normalize();
	[[nodiscard]] Vec2 Normalized() const;
	[[nodiscard]] Vec2 UnitVector() const;
	[[nodiscard]] Vec2 Perpendicular() const;

	static Vec2 Zero();

	[[nodiscard]] float Dot(const Vec2& v) const;
	[[nodiscard]] float Cross(const Vec2& v) const;

	bool operator == (const Vec2& v) const;
	bool operator != (const Vec2& v) const;

	Vec2 operator + (const Vec2& v) const;
	Vec2 operator - (const Vec2& v) const;
	Vec2 operator * (float n) const;
	Vec2 operator / (float n) const;
	Vec2 operator - () const;

	Vec2& operator += (const Vec2& v);
	Vec2& operator -= (const Vec2& v);
	Vec2& operator *= (float n);
	Vec2& operator /= (float n);
};