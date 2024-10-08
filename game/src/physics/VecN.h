#pragma once

class VecN
{
public:
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
	VecN operator *(float n) const;
	const VecN& operator +=(const VecN& v);
	const VecN& operator -=(const VecN& v);
	const VecN& operator *=(float n);
	float operator [](int index) const;
	float& operator [](int index);

	[[nodiscard]] int N() const
	{
		return m_n;
	}

private:
	int m_n;
	float* m_data;
};
