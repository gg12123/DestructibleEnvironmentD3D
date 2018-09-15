#pragma once
#include <array>
#include "Vector3.h"
#include "Quaternion.h"

template<int Size>
class Matrix
{
public:
	Matrix()
	{
		for (int i = 0; i < Size; i++)
			memset(M[i], 0, Size * sizeof(float));
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

	template <typename... T>
	void SetColumn(int col, T... dataVals)
	{
		float data[]{ dataVals... };

		auto c = M[col];

		for (int i = 0; i < Size; i++)
			c[i] = data[i];
	}

	// static

	static inline Matrix<Size> Indentity()
	{
		Matrix<Size> m;

		for (int i = 0; i < Size; i++)
			m.M[i][i] = 1.0f;

		return m;
	}

	static inline Matrix<Size> FromRotation(const Quaternion& rot)
	{
		//static_assert(Size >= 3);
		auto res = Matrix<Size>::Indentity();

		auto qr = rot.r;
		auto qi = rot.x;
		auto qj = rot.y;
		auto qk = rot.z;

		auto qj2 = qj * qj;
		auto qk2 = qk * qk;
		auto qi2 = qi * qi;

		//auto col0 = res.M[0];
		//col0[0] = 1.0f - 2.0f * (qj2 + qk2);
		//col0[1] = 2.0f * (qi * qj - qk * qr);
		//col0[2] = 2.0f * (qi * qk + qj * qr);
		//
		//auto col1 = res.M[1];
		//col1[0] = 2.0f * (qi * qj + qk * qr);
		//col1[1] = 1.0f - 2.0f * (qi2 + qk2);
		//col1[2] = 2.0f * (qj * qk - qi * qr);
		//
		//auto col2 = res.M[2];
		//col2[0] = 2.0f * (qi * qk - qj * qr);
		//col2[1] = 2.0f * (qj * qk + qi * qr);
		//col2[2] = 1.0f - 2.0f * (qi2 + qj2);

		auto col0 = res.M[0];
		col0[0] = 1.0f - 2.0f * (qj2 + qk2);
		col0[1] = 2.0f * (qi * qj + qk * qr);
		col0[2] = 2.0f * (qi * qk - qj * qr);

		auto col1 = res.M[1];
		col1[0] = 2.0f * (qi * qj - qk * qr);
		col1[1] = 1.0f - 2.0f * (qi2 + qk2);
		col1[2] = 2.0f * (qj * qk + qi * qr);

		auto col2 = res.M[2];
		col2[0] = 2.0f * (qi * qk + qj * qr);
		col2[1] = 2.0f * (qj * qk - qi * qr);
		col2[2] = 1.0f - 2.0f * (qi2 + qj2);

		return res;
	}

	static inline Matrix<4> FromTranslation(const Vector3& tran)
	{
		auto res = Matrix<4>::Indentity();

		auto col3 = res.M[3];

		col3[0] = tran.x;
		col3[1] = tran.y;
		col3[2] = tran.z;

		return res;
	}

	static inline Matrix<3> Inverse(const Matrix<3>& mat)
	{
		auto col0 = mat.M[0];
		auto e11 = col0[0];
		auto e21 = col0[1];
		auto e31 = col0[2];

		auto col1 = mat.M[1];
		auto e12 = col1[0];
		auto e22 = col1[1];
		auto e32 = col1[2];

		auto col2 = mat.M[2];
		auto e13 = col2[0];
		auto e23 = col2[1];
		auto e33 = col2[2];

		auto d = e11 * e22 * e33 -
			e11 * e32 * e23 +
			e21 * e32 * e13 -
			e21 * e12 * e33 +
			e31 * e12 * e23 -
			e31 * e22 * e13;

		Matrix3 inv;

		auto invCol0 = inv.M[0];
		invCol0[0] = (e22 * e33 - e23 * e32) / d;
		invCol0[1] = -(e21 * e33 - e23 * e31) / d;
		invCol0[2] = (e21 * e32 - e22 * e31) / d;

		auto invCol1 = inv.M[1];
		invCol1[0] = -(e12 * e33 - e13 * e32) / d;
		invCol1[1] = (e11 * e33 - e13 * e31) / d;
		invCol1[2] = -(e11 * e32 - e12 * e31) / d;

		auto invCol2 = inv.M[2];
		invCol2[0] = (e12 * e23 - e13 * e22) / d;
		invCol2[1] = -(e11 * e23 - e13 * e21) / d;
		invCol2[2] = (e11 * e22 - e12 * e21) / d;

		return inv;
	}

	static inline Vector3 MatrixVectorMul3(const Matrix<Size>& lhs, const Vector3& rhs)
	{
		//static_assert(Size >= 3);

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
		auto q = 1.0f / tanf(0.5f * fov);
		auto A = q / aspect;
		auto B = (-nearClip - farClip) / (nearClip - farClip);
		auto C = (2.0f * nearClip * farClip) / (nearClip - farClip);

		Matrix<4> result;

		result.SetColumn(0, A, 0.0f, 0.0f, 0.0f);
		result.SetColumn(1, 0.0f, q, 0.0f, 0.0f);
		result.SetColumn(2, 0.0f, 0.0f, B, 1.0f);
		result.SetColumn(3, 0.0f, 0.0f, C, 0.0f);

		return result;
	}
};

template<int N>
Matrix<N> operator*(const Matrix<N>& lhs, const Matrix<N>& rhs)
{
	Matrix<N> result;

	for (int i = 0; i < N; i++)
	{
		auto iColB = rhs.M[i];
		auto iColC = result.M[i];

		for (int j = 0; j < N; j++)
		{
			auto jColA = lhs.M[j];
			auto ijB = iColB[j];

			for (int k = 0; k < N; k++)
			{
				iColC[k] += ijB * jColA[k];
			}
		}
	}

	return result;
}

//template<int N>
//Matrix<N> operator*(const Matrix<N>& lhs, const Matrix<N>& rhs)
//{
//	Matrix<N> result;
//
//	for (int j = 0; j < N; j++)
//	{
//		for (int i = 0; i < N; i++)
//		{
//			auto sum = 0.0f;
//
//			for (int n = 0; n < N; n++)
//			{
//				sum += lhs.M[n][i] * rhs.M[j][n]; // TODO - maybe transposing one of them first will be faster?
//			}
//			result.M[j][i] = sum;
//		}
//	}
//	return result;
//}

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