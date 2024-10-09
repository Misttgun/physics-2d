#include "VecN.h"

#include <utility>

VecN::VecN(): n(0), data(nullptr) {}

VecN::VecN(const int n): n(n)
{
	data = new float[n];
}

VecN::VecN(const VecN& v)
{
	n = v.n;
	data = new float[n];
	for (int i = 0; i < n; i++)
		data[i] = v.data[i];
}

VecN::~VecN()
{
	delete[] data;
}

VecN::VecN(VecN&& v) noexcept : n(v.n)
{
	data = v.data;
	v.data = nullptr;
}

VecN& VecN::operator=(VecN&& v) noexcept
{
	delete[] data;
	data = v.data;
	n = v.n;

	v.data = nullptr;
	return *this;
}

void VecN::Zero() const
{
	for (int i = 0; i < n; i++)
		data[i] = 0.0f;
}

float VecN::Dot(const VecN& v) const
{
	float sum = 0.0f;
	for (int i = 0; i < n; i++)
		sum += data[i] * v.data[i];
	return sum;
}

VecN& VecN::operator =(const VecN& v)
{
	VecN temp = v;
	*this = std::move(temp);
	return *this;
}

VecN VecN::operator *(const float value) const
{
	VecN result = *this;
	result *= value;
	return result;
}

VecN VecN::operator +(const VecN& v) const
{
	VecN result = *this;
	for (int i = 0; i < n; i++)
		result.data[i] += v.data[i];
	return result;
}

VecN VecN::operator -(const VecN& v) const
{
	VecN result = *this;
	for (int i = 0; i < n; i++)
		result.data[i] -= v.data[i];
	return result;
}

const VecN& VecN::operator *=(const float value)
{
	for (int i = 0; i < n; i++)
		data[i] *= value;
	return *this;
}

const VecN& VecN::operator +=(const VecN& v)
{
	for (int i = 0; i < n; i++)
		data[i] += v.data[i];
	return *this;
}

const VecN& VecN::operator -=(const VecN& v)
{
	for (int i = 0; i < n; i++)
		data[i] -= v.data[i];
	return *this;
}

float VecN::operator [](const int index) const
{
	return data[index];
}

float& VecN::operator [](const int index)
{
	return data[index];
}
