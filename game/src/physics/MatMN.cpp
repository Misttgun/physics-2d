#include "MatMN.h"

#include <utility>

MatMN::MatMN(): m_m(0), m_n(0), m_rows(nullptr) {}

MatMN::MatMN(const int m, const int n): m_m(m), m_n(n)
{
	m_rows = new VecN[m];

	for (int i = 0; i < m; i++)
		m_rows[i] = VecN(n);
}

MatMN::MatMN(const MatMN& m) : m_m(m.m_m), m_n(m.m_n)
{
	m_rows = new VecN[m_m];
	for (int i = 0; i < m_m; i++)
		m_rows[i] = m.m_rows[i];
}

MatMN::~MatMN()
{
	delete[] m_rows;
}

MatMN::MatMN(MatMN&& m) noexcept : m_m(m.m_m), m_n(m.m_n)
{
	m_rows = m.m_rows;
	m.m_rows = nullptr;
}

MatMN& MatMN::operator=(MatMN&& m) noexcept
{
	delete[] m_rows;

	m_rows = m.m_rows;
	m_n = m.m_n;
	m_m = m.m_m;

	m_rows = nullptr;
	return *this;
}

void MatMN::Zero() const
{
	for (int i = 0; i < m_m; i++)
		m_rows[i].Zero();
}

MatMN MatMN::Transpose() const
{
	MatMN result(m_n, m_m);

	for (int i = 0; i < m_m; i++)
		for (int j = 0; j < m_n; j++)
			result.m_rows[j][i] = m_rows[i][j];

	return result;
}

MatMN& MatMN::operator =(const MatMN& m)
{
	MatMN temp = m;
	*this = std::move(temp);
	return *this;
}

VecN MatMN::operator *(const VecN& v) const
{
	if (v.N() != m_n)
		return v;

	VecN result(m_m);

	for (int i = 0; i < m_m; i++)
		result[i] = v.Dot(m_rows[i]);

	return result;
}

MatMN MatMN::operator *(const MatMN& m) const
{
	if (m.m_m != m_n && m.m_n != m_m)
		return m;

	const MatMN transposed = m.Transpose();
	MatMN result(m_m, m.m_n);

	for (int i = 0; i < m_m; i++)
		for (int j = 0; j < m.m_n; j++)
			result.m_rows[i][j] = m_rows[i].Dot(transposed.m_rows[j]);

	return result;
}
