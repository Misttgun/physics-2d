#include "MatMN.h"

#include <utility>

MatMN::MatMN(): m(0), n(0), rows(nullptr) {}

MatMN::MatMN(const int row, const int col): m(row), n(col)
{
	rows = new VecN[m];

	for (int i = 0; i < m; i++)
		rows[i] = VecN(n);
}

MatMN::MatMN(const MatMN& mat) : m(mat.m), n(mat.n)
{
	rows = new VecN[m];
	for (int i = 0; i < m; i++)
		rows[i] = mat.rows[i];
}

MatMN::~MatMN()
{
	delete[] rows;
}

MatMN::MatMN(MatMN&& mat) noexcept : m(mat.m), n(mat.n)
{
	rows = mat.rows;
	mat.rows = nullptr;
}

MatMN& MatMN::operator=(MatMN&& mat) noexcept
{
	delete[] rows;

	rows = mat.rows;
	n = mat.n;
	m = mat.m;

	rows = nullptr;
	return *this;
}

void MatMN::Zero() const
{
	for (int i = 0; i < m; i++)
		rows[i].Zero();
}

MatMN MatMN::Transpose() const
{
	MatMN result(n, m);

	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			result.rows[j][i] = rows[i][j];

	return result;
}

MatMN& MatMN::operator =(const MatMN& mat)
{
	MatMN temp = mat;
	*this = std::move(temp);
	return *this;
}

VecN MatMN::operator *(const VecN& v) const
{
	if (v.n != n)
		return v;

	VecN result(m);

	for (int i = 0; i < m; i++)
		result[i] = v.Dot(rows[i]);

	return result;
}

MatMN MatMN::operator *(const MatMN& mat) const
{
	if (mat.m != n && mat.n != m)
		return mat;

	const MatMN transposed = mat.Transpose();
	MatMN result(m, mat.n);

	for (int i = 0; i < m; i++)
		for (int j = 0; j < mat.n; j++)
			result.rows[i][j] = rows[i].Dot(transposed.rows[j]);

	return result;
}

VecN MatMN::SolveGaussSeidel(const MatMN& mat, const VecN& vec)
{
	const int vN = vec.n;
	VecN x(vN);
	x.Zero();

	// Iterate N times
	for (int iterations = 0; iterations < vN; iterations++)
	{
		for (int i = 0; i < vN; i++)
		{
			const float dx = (vec[i] / mat.rows[i][i]) - (mat.rows[i].Dot(x) / mat.rows[i][i]);

			if (isnan(dx) == false)
				x[i] += dx;
		}
	}
	return x;
}
