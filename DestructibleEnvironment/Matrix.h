#pragma once
#include "Vector.h"

template<int Size>
class Matrix
{
public:
	float M[Size][Size]; // [col][row]

	Matrix<Size> operator*(const Matrix<Size>& rhs) const
	{
		Matrix<Size> result;

		for (int j = 0; j < Size; j++)
		{
			for (int i = 0; i < Size; i++)
			{
				auto sum = 0.0f;

				for (int n = 0; n < Size; n++)
				{
					sum += M[n][i] * rhs.M[j][n]; // TODO - maybe transposing one of them first will be faster?
				}
				result[j][i] = sum;
			}
		}
	}

protected:
	static inline Vector3 MulAssume3D(const Vector3& rhs)
	{
		auto res = Vector3::Zero();

		auto col0 = M[0];

		res.x += col0[0] * rhs.x;
		res.y += col0[1] * rhs.x;
		res.z += col0[2] * rhs.x;

		auto col1 = M[1];

		res.x += col1[0] * rhs.y;
		res.y += col1[1] * rhs.y;
		res.z += col1[2] * rhs.y;

		auto col2 = M[2];

		res.x += col2[0] * rhs.z;
		res.y += col2[1] * rhs.z;
		res.z += col2[2] * rhs.z;

		return res;
	}
};

class Matrix3 : public Matrix<3>
{
public:
	Vector3 operator*(const Vector3& rhs) const
	{
		return MulAssume3D(rhs);
	}
};

class Matrix4 : public Matrix<4>
{
public:
	Vector3 operator*(const Vector3& rhs) const
	{
		auto res = MulAssume3D(rhs);

		auto col3 = M[3];

		res.x += col3[0];
		res.y += col3[1];
		res.z += col3[2];

		return res;
	}
};