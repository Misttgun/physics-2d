#include "physics/Vec2.h"
#include <cmath>

Vec2::Vec2() : x(0.0), y(0.0)
{}

Vec2::Vec2(const float x, const float y) : x(x), y(y)
{}

Vec2 Vec2::Rotate(const float angle) const
{
	Vec2 result;
	result.x = x * cos(angle) - y * sin(angle);
	result.y = x * sin(angle) + y * cos(angle);
	return result;
}

float Vec2::Magnitude() const
{
	return sqrtf(x * x + y * y);
}

float Vec2::MagnitudeSquared() const
{
	return (x * x + y * y);
}

Vec2& Vec2::Normalize()
{
	const float length = Magnitude();
	if (length != 0.0f)
	{
		x /= length;
		y /= length;
	}

	return *this;
}

Vec2 Vec2::Normalized() const
{
	Vec2 result = Vec2(0, 0);
	const float length = Magnitude();

	if (length != 0.0f)
	{
		result.x = x / length;
		result.y = y / length;
	}

	return result;
}

Vec2 Vec2::UnitVector() const
{
	Vec2 result = Vec2(0, 0);
	const float length = Magnitude();

	if (length != 0.0f)
	{
		result.x = x / length;
		result.y = y / length;

	}

	return result;
}

Vec2 Vec2::Perpendicular() const
{
	return Vec2(y, -x).Normalize();
}

Vec2 Vec2::Zero()
{
	return {0.0f, 0.0f};
}

float Vec2::Dot(const Vec2& v) const
{
	return (x * v.x) + (y * v.y);
}

float Vec2::Cross(const Vec2& v) const
{
	return (x * v.y) - (y * v.x);
}

bool Vec2::operator == (const Vec2& v) const
{
	return abs(x - v.x) < 0.001f && abs(y - v.y) < 0.001f;
}

bool Vec2::operator != (const Vec2& v) const
{
	return !(*this == v);
}

Vec2 Vec2::operator + (const Vec2& v) const
{
	Vec2 result;
	result.x = x + v.x;
	result.y = y + v.y;
	return result;
}

Vec2 Vec2::operator - (const Vec2& v) const
{
	Vec2 result;
	result.x = x - v.x;
	result.y = y - v.y;
	return result;
}

Vec2 Vec2::operator * (const float n) const
{
	Vec2 result;
	result.x = x * n;
	result.y = y * n;
	return result;
}

Vec2 Vec2::operator / (const float n) const
{
	Vec2 result;
	result.x = x / n;
	result.y = y / n;
	return result;
}

Vec2& Vec2::operator += (const Vec2& v)
{
	x += v.x;
	y += v.y;
	return *this;
}

Vec2& Vec2::operator -= (const Vec2& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

Vec2& Vec2::operator *= (const float n)
{
	x *= n;
	y *= n;
	return *this;
}

Vec2& Vec2::operator /= (const float n)
{
	x /= n;
	y /= n;
	return *this;
}

Vec2 Vec2::operator - () const
{
	Vec2 result;
	result.x = x * -1;
	result.y = y * -1;
	return result;
}
