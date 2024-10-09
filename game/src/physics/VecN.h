#pragma once

struct VecN
{
	VecN();
	explicit VecN(int n);
	VecN(const VecN& v);
	~VecN();

	VecN(VecN&& v) noexcept;
	VecN& operator=(VecN&& v) noexcept;

	void Zero() const;
	[[nodiscard]] float Dot(const VecN& v) const;

	VecN& operator =(const VecN& v);
	VecN operator +(const VecN& v) const;
	VecN operator -(const VecN& v) const;
	VecN operator *(float value) const;
	const VecN& operator +=(const VecN& v);
	const VecN& operator -=(const VecN& v);
	const VecN& operator *=(float value);
	float operator [](int index) const;
	float& operator [](int index);

	int n;
	float* data;
};
