#include "VecN.h"

#include <utility>

VecN::VecN(): m_n(0), m_data(nullptr) {}

VecN::VecN(const int n): m_n(n)
{
	m_data = new float[n];
}

VecN::VecN(const VecN& v)
{
	m_n = v.m_n;
	m_data = new float[m_n];
	for (int i = 0; i < m_n; i++)
		m_data[i] = v.m_data[i];
}

VecN::~VecN()
{
	delete[] m_data;
}

VecN::VecN(VecN&& v) noexcept : m_n(v.m_n)
{
	m_data = v.m_data;
	v.m_data = nullptr;
}

VecN& VecN::operator=(VecN&& v) noexcept
{
	delete[] m_data;
	m_data = v.m_data;
	m_n = v.m_n;

	v.m_data = nullptr;
	return *this;
}

void VecN::Zero() const
{
	for (int i = 0; i < m_n; i++)
		m_data[i] = 0.0f;
}

float VecN::Dot(const VecN& v) const
{
	float sum = 0.0f;
	for (int i = 0; i < m_n; i++)
		sum += m_data[i] * v.m_data[i];
	return sum;
}

VecN& VecN::operator =(const VecN& v)
{
	VecN temp = v;
	*this = std::move(temp);
	return *this;
}

VecN VecN::operator *(const float n) const
{
	VecN result = *this;
	result *= n;
	return result;
}

VecN VecN::operator +(const VecN& v) const
{
	VecN result = *this;
	for (int i = 0; i < m_n; i++)
		result.m_data[i] += v.m_data[i];
	return result;
}

VecN VecN::operator -(const VecN& v) const
{
	VecN result = *this;
	for (int i = 0; i < m_n; i++)
		result.m_data[i] -= v.m_data[i];
	return result;
}

const VecN& VecN::operator *=(const float n)
{
	for (int i = 0; i < m_n; i++)
		m_data[i] *= n;
	return *this;
}

const VecN& VecN::operator +=(const VecN& v)
{
	for (int i = 0; i < m_n; i++)
		m_data[i] += v.m_data[i];
	return *this;
}

const VecN& VecN::operator -=(const VecN& v)
{
	for (int i = 0; i < m_n; i++)
		m_data[i] -= v.m_data[i];
	return *this;
}

float VecN::operator [](const int index) const
{
	return m_data[index];
}

float& VecN::operator [](const int index)
{
	return m_data[index];
}
