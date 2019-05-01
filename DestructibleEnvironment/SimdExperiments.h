#pragma once
#include "Vector3.h"
#include "Matrix.h"
#include "Debug.h"
#include "FixedTimeStepTime.h"

class SimdExperiments
{
private:
	static void Normal()
	{
		auto a = Vector3(1.0f, -2.0f, 4.0f);
		auto b = Vector3(2.0f, -2.0f, 5.0f);

		auto c = Vector3::Cross(a, b);
		Debug::Log(c.x);
		Debug::Log(c.y);
		Debug::Log(c.z);
	}

	static void Simd()
	{
		SimdMatrix<4> M0;
		SimdMatrix<4> M1;

		M0.Cols[0] = SimdVector3(1.0f, 2.0f, 3.0f);
		M0.Cols[1] = SimdVector3(1.0f, 2.0f, 3.0f);
		M0.Cols[2] = SimdVector3(1.0f, 2.0f, 3.0f);

		M1.Cols[0] = SimdVector3(1.0f, 2.0f, 3.0f);
		M1.Cols[1] = SimdVector3(1.0f, 2.0f, 3.0f);
		M1.Cols[2] = SimdVector3(1.0f, 2.0f, 3.0f);

		auto r = M0 * M1;
		auto i = 0;
	}

public:
	static void Run()
	{
		Normal();
		Simd();
	}
};
