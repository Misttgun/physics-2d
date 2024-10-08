#pragma once

#include "VecN.h"

class MatMN
{
public:
	MatMN();
	MatMN(int m, int n);
	MatMN(const MatMN& m);
	~MatMN();

	MatMN(MatMN&& m) noexcept;
	MatMN& operator =(MatMN&& m) noexcept;

	void Zero() const;
	[[nodiscard]] MatMN Transpose() const;

	MatMN& operator =(const MatMN& m);
	VecN operator *(const VecN& v) const;
	MatMN operator *(const MatMN& m) const;

private:
	int m_m;
	int m_n;

	VecN* m_rows; // the rows of the matrix with N columns inside
};
