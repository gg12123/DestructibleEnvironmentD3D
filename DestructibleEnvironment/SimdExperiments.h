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
		auto a = Vector3(1.0f, 2.0f, 3.0f);
		auto b = Vector3(1.0f, 0.0f, 2.0f);
		auto ab = Vector3::Cross(a, b);

		auto aa = SimdVector3(1.0f, 2.0f, 3.0f);
		auto bb = SimdVector3(1.0f, 0.0f, 2.0f);
		auto ab2 = SimdVector3::Cross(aa, bb);

		Normal();
		Simd();
	}
};
