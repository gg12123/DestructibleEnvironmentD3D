#include "pch.h"
#include "Rigidbody.h"
#include "PhysicsTime.h"
#include "MathU.h"
#include "Shape.h"
#include "Face.h"
#include "ShapePoint.h"
#include "Debug.h"

static void SubExpressions(float w0, float w1, float w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2)
{
	auto temp0 = w0 + w1;
	f1 = temp0 + w2;
	auto temp1 = w0 * w0;
	auto temp2 = temp1 + w1 * temp0;
	f2 = temp2 + w2 * f1;
	f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
	g0 = f2 + w0 * (f1 + w0);
	g1 = f2 + w1 * (f1 + w1);
	g2 = f2 + w2 * (f1 + w2);
}

void Rigidbody::InitMassProperties(const Transform& refTran)
{
	float f1x, f2x, f3x, g0x, g1x, g2x;
	float f1y, f2y, f3y, g0y, g1y, g2y;
	float f1z, f2z, f3z, g0z, g1z, g2z;

	static float intg[10];
	std::memset(intg, 0, 10 * sizeof(float));

	static float mult[] = { 1.0f / 6.0f, 1.0f / 24.0f, 1.0f / 24.0f, 1.0f / 24.0f,
		1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 60.0f, 1 / 120.0f, 1 / 120.0f , 1 / 120.0f };

	for (auto s : GetSubShapes())
	{
		for (auto f : s->GetFaces())
		{
			auto& points = f->GetPointObjects();
			auto p0 = points[0]->GetPoint();

			for (auto i = 1u; i < points.size() - 1u; i++)
			{
				auto p1 = points[i]->GetPoint();
				auto p2 = points[i + 1u]->GetPoint();

				auto x0 = p0.X();
				auto y0 = p0.Y();
				auto z0 = p0.Z();

				auto x1 = p1.X();
				auto y1 = p1.Y();
				auto z1 = p1.Z();

				auto x2 = p2.X();
				auto y2 = p2.Y();
				auto z2 = p2.Z();

				auto a1 = x1 - x0;
				auto b1 = y1 - y0;
				auto c1 = z1 - z0;

				auto a2 = x2 - x0;
				auto b2 = y2 - y0;
				auto c2 = z2 - z0;

				auto d0 = b1 * c2 - b2 * c1;
				auto d1 = a2 * c1 - a1 * c2;
				auto d2 = a1 * b2 - a2 * b1;

				SubExpressions(x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x);
				SubExpressions(y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y);
				SubExpressions(z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z);

				intg[0] += d0 * f1x;
				intg[1] += d0 * f2x;
				intg[2] += d1 * f2y;
				intg[3] += d2 * f2z;
				intg[4] += d0 * f3x;
				intg[5] += d1 * f3y;
				intg[6] += d2 * f3z;
				intg[7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
				intg[8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
				intg[9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
			}
		}
	}

	for (auto i = 0; i < 10; i++)
		intg[i] *= mult[i];

	auto mass = intg[0];
	auto cm = Vector3(intg[1] / mass, intg[2] / mass, intg[3] / mass);

	auto Ixx = (intg[5] + intg[6] - mass * (cm.Y() * cm.Y() + cm.Z() * cm.Z()));
	auto Iyy = (intg[4] + intg[6] - mass * (cm.Z() * cm.Z() + cm.X() * cm.X()));
	auto Izz = (intg[4] + intg[5] - mass * (cm.X() * cm.X() + cm.Y() * cm.Y()));
	auto Ixy = (-(intg[7] - mass * cm.X() * cm.Y()));
	auto Iyz = (-(intg[8] - mass * cm.Y() * cm.Z()));
	auto Ixz = (-(intg[9] - mass * cm.Z() * cm.X()));

	// TODO - enforcing a min inertia and mass keeps the simulation stable but the
	// dynamics do not look right for tiny objects - find another way!

	static constexpr auto minInertia = 0.01f;
	auto Imin = MathU::Min(MathU::Min(Ixx, Iyy), Izz);
	if (Imin < minInertia)
	{
		auto multi = minInertia / Imin;
		Ixx *= multi;
		Iyy *= multi;
		Izz *= multi;
		Ixy *= multi;
		Iyz *= multi;
		Ixz *= multi;
	}
	
	static constexpr auto minMass = 0.1f;
	mass = MathU::Max(mass, minMass);

	Matrix3 inertia;
	auto col0 = inertia.Cols[0].Floats;
	col0[0] = Ixx;
	col0[1] = -Ixy;
	col0[2] = -Ixz;
	
	auto col1 = inertia.Cols[1].Floats;
	col1[0] = -Ixy;
	col1[1] = Iyy;
	col1[2] = -Iyz;
	
	auto col2 = inertia.Cols[2].Floats;
	col2[0] = -Ixz;
	col2[1] = -Iyz;
	col2[2] = Izz;

	SetMass(mass);
	SetInertia(inertia);
	CentreAndCache(refTran, cm);
}

bool Rigidbody::IsStill() const
{
	return false;
	//return m_StillnessMonitor.IsStill();
}

void Rigidbody::UpdateTransform()
{
	auto& t = GetTransform();
	auto& q = t.GetRotation();

	auto pos = t.GetPosition() + m_VelocityWorld * PhysicsTime::FixedDeltaTime;
	auto rot = q + m_AngularVelocityWorld * q * 0.5f * PhysicsTime::FixedDeltaTime;
	
	t.SetPositionAndRotation(pos, rot);
}

void Rigidbody::ApplyImpulse(const Impulse& impulse)
{
	WakeUp();

	m_VelocityWorld += GetInvMass() * impulse.WorldImpulse;

	auto& r = impulse.WorldImpulsePoint - GetTransform().GetPosition();
	auto& J = impulse.WorldImpulse;

	// TODO - cache the inertia inverse. The get call re-calculates it each
	// time and this method is called loads by the solver.
	m_AngularVelocityWorld += (GetInertiaInverseWorld() * Vector3::Cross(r, J));
}

void Rigidbody::ApplyExternalForcesAndImpulses()
{
	static constexpr float g = 9.8f;

	m_ExternalForceWorld -= GetMass() * g * Vector3::Up();
	m_ExternalForceWorld -= m_Drag * m_VelocityWorld;

	m_ExternalMomentsWorld -= m_AngularDrag * m_AngularVelocityWorld;

	m_VelocityWorld += (GetInvMass() * m_ExternalForceWorld) * PhysicsTime::FixedDeltaTime;
	m_AngularVelocityWorld += (GetInertiaInverseWorld() * m_ExternalMomentsWorld) * PhysicsTime::FixedDeltaTime;

	m_ExternalForceWorld = Vector3::Zero();
	m_ExternalMomentsWorld = Vector3::Zero();
}

void Rigidbody::UpdatePosition()
{
	UpdateTransform();
	UpdateWorldInertiaInverse();
	UpdateSubShapesWorldAABBs();
	m_StillnessMonitor.Tick(m_VelocityWorld, m_AngularVelocityWorld);
}