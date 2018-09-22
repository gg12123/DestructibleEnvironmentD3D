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

	Matrix<3> ToMatrix3()
	{
		Matrix3 mat3;
		auto n = MathUtils::Min(3, Size);

		for (auto i = 0; i < n; i++)
		{
			auto colOut = mat3.M[i];
			auto colIn = M[i];

			for (auto j = 0; j < n; j++)
				colOut[j] = colIn[j];
		}

		return mat3;
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
		auto a = col0[0];
		auto d = col0[1];
		auto g = col0[2];

		auto col1 = mat.M[1];
		auto b = col1[0];
		auto e = col1[1];
		auto h = col1[2];

		auto col2 = mat.M[2];
		auto c = col2[0];
		auto f = col2[1];
		auto i = col2[2];

		auto A = e * i - f * h;
		auto B = -(d * i - f * g);
		auto C = d * h - e * g;

		auto D = -(b * i - c * h);
		auto E = (a * i - c * g);
		auto F = -(a * h - b * g);

		auto G = b * f - c * e;
		auto H = -(a * f - c * d);
		auto I = a * e - b * d;

		auto det = (a * A + b * B + c * C);
		assert(det != 0.0f);

		auto invDet = 1.0f / det;

		Matrix3 inv;

		auto invCol0 = inv.M[0];
		invCol0[0] = A * invDet;
		invCol0[1] = B * invDet;
		invCol0[2] = C * invDet;

		auto invCol1 = inv.M[1];
		invCol1[0] = D * invDet;
		invCol1[1] = E * invDet;
		invCol1[2] = F * invDet;

		auto invCol2 = inv.M[2];
		invCol2[0] = G * invDet;
		invCol2[1] = H * invDet;
		invCol2[2] = I * invDet;

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