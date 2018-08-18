#pragma once
#include "Vector.h"
#include "Quaternion.h"

template<int Size>
class Matrix
{
public:
	Matrix()
	{
	}

	Matrix(float val)
	{
		for (int i = 0; i < Size; i++)
			for (int j = 0; j < Size; j++)
				M[i][j] = val;
	}

	float M[Size][Size]; // [col][row]

	Matrix<Size> Transposed()
	{
		Matrix<Size> res;

		for (int i = 0; i < Size; i++)
			for (int j = 0; j < Size; j++)
				res.M[i][j] = M[j][i]; // TODO - maybe this isnt the most efficient way to access the memory?

		return res;
	}

	void SetColumn(int col, const std::array<float, Size>& data)
	{
		auto c = M[col];

		for (int i = 0; i < Size; i++)
			c[i] = data[i];
	}

	// static

	static inline Matrix<Size> Indentity()
	{
		Matrix<Size> m(0.0f);

		for (int i = 0; i < Size; i++)
			m[i][i] = 1.0f;

		return m;
	}

	static inline Matrix<Size> FromRotation(const Quaternion& rot)
	{
		static_assert(Size >= 3);
		auto res = Matrix<Size>::Indentity();
		// fill in the top left 3 by 3 part
		return res;
	}

	static inline Matrix<4> FromTranslation(const Vector3& tran)
	{
		auto res = Matrix<4>::Indentity();

		auto col3 = M[3];

		col3[0] = tran.x;
		col3[1] = tran.x;
		col3[2] = tran.z;

		return res;
	}

	static inline Matrix<3> Inverse(const Matrix<3>& mat)
	{

	}

	static inline Vector3 MatrixVectorMul3(const Matrix<Size>& lhs, const Vector3& rhs)
	{
		static_assert(N >= 3);

		auto res = Vector3::Zero();

		auto col0 = lhs.M[0];

		res.x += col0[0] * rhs.x;
		res.y += col0[1] * rhs.x;
		res.z += col0[2] * rhs.x;

		auto col1 = lhs.M[1];

		res.x += col1[0] * rhs.y;
		res.y += col1[1] * rhs.y;
		res.z += col1[2] * rhs.y;

		auto col2 = lhs.M[2];

		res.x += col2[0] * rhs.z;
		res.y += col2[1] * rhs.z;
		res.z += col2[2] * rhs.z;

		return res;
	}

	static inline Matrix<4> Perspective(float fov, float aspect, float nearClip, float farClip)
	{
		auto q = 1.0f / tanf(0.5f * fovy);
		auto A = q / aspect;
		auto B = (-n - f) / (n - f);
		auto C = (2.0f * n * f) / (n - f);

		Matrix<4> result;

		result.SetColumn(0, {A, 0.0f, 0.0f, 0.0f});
		result.SetColumn(1, {0.0f, q, 0.0f, 0.0f});
		result.SetColumn(2, {0.0f, 0.0f, B, 1.0f});
		result.SetColumn(3, {0.0f, 0.0f, C, 0.0f});

		return result;
	}
};

template<int N>
Matrix<N> operator*(const Matrix<N>& lhs, const Matrix<N>& rhs) const
{
	Matrix<Size> result;

	for (int j = 0; j < Size; j++)
	{
		for (int i = 0; i < Size; i++)
		{
			auto sum = 0.0f;

			for (int n = 0; n < Size; n++)
			{
				sum += lhs.M[n][i] * rhs.M[j][n]; // TODO - maybe transposing one of them first will be faster?
			}
			result.M[j][i] = sum;
		}
	}
	return result;
}

using Matrix3 = Matrix<3>;
using Matrix4 = Matrix<4>;


static inline Vector3 operator*(const Matrix3& lhs, const Vector3& rhs)
{
	return Matrix3::MatrixVectorMul3(lhs, rhs);
}

static inline Vector3 operator*(const Matrix4& lhs, const Vector3& rhs)
{
	auto res = Matrix4::MatrixVectorMul3(lhs, rhs);

	auto col3 = lhs.M[3];

	res.x += col3[0];
	res.y += col3[1];
	res.z += col3[2];

	return res;
}