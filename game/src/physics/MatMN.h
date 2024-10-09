#pragma once

#include "VecN.h"

struct MatMN
{
	int m;
	int n;
	VecN* rows; // the rows of the matrix with N columns inside

	MatMN();
	MatMN(int row, int col);
	MatMN(const MatMN& mat);
	~MatMN();

	MatMN(MatMN&& mat) noexcept;
	MatMN& operator =(MatMN&& mat) noexcept;

	void Zero() const;
	[[nodiscard]] MatMN Transpose() const;

	MatMN& operator =(const MatMN& mat);
	VecN operator *(const VecN& v) const;
	MatMN operator *(const MatMN& mat) const;

	static VecN SolveGaussSeidel(const MatMN& mat, const VecN& vec);
};
