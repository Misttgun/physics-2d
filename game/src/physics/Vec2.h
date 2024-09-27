#pragma once

struct Vec2
{
	float x;
	float y;

	Vec2();
	Vec2(float x, float y);

	~Vec2();

	Vec2 Rotate(float angle) const;
	float Magnitude() const;
	float MagnitudeSquared() const;

	Vec2& Normalize();
	Vec2 Normalized() const;
	Vec2 UnitVector() const;
	Vec2 Normal() const;

	static Vec2 Zero();

	float Dot(const Vec2& v) const;
	float Cross(const Vec2& v) const;

	Vec2& operator = (const Vec2& v);
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