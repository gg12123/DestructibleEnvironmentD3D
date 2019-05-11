#pragma once
#include "Shape.h"
#include "CollisionData.h"
#include "Face.h"
#include "PhysicsTime.h"
#include "Debug.h"

class PhysicsObject;

class VelocityAtPointConstraint
{
private:
	static float CalculateS(const Vector3& n, const Vector3& r, const Matrix3 inertiaInverse)
	{
		auto x = inertiaInverse * Vector3::Cross(r, n);
		return Vector3::Dot(n, Vector3::Cross(x, r));
	}

	void RecalculateDenom();

public:
	VelocityAtPointConstraint(PhysicsObject& body1, PhysicsObject& body2, const Vector3& direction, const Vector3& point, float velBias)
		: m_Body1(&body1), m_Body2(&body2), m_Direction(direction), m_Point(point), m_AcumulatedImpulse(0.0f), m_VBias(velBias)
	{
		RecalculateDenom();
	}

	VelocityAtPointConstraint(PhysicsObject& body1, PhysicsObject& body2, float velBias)
		: m_Body1(&body1), m_Body2(&body2), m_AcumulatedImpulse(0.0f), m_VBias(velBias)
	{
		RecalculateDenom();
	}

	void ResetPointAndDirection(const Vector3& p, const Vector3& dir)
	{
		m_Point = p;
		m_Direction = dir;
		RecalculateDenom();
	}

	auto& GetBody1() const
	{
		return *m_Body1;
	}

	auto& GetBody2() const
	{
		return *m_Body2;
	}

	const auto& GetPoint() const
	{
		return m_Point;
	}

	const auto& GetDirection() const
	{
		return m_Direction;
	}

	float GetAccumulatedImpulse() const
	{
		return m_AcumulatedImpulse;
	}

	void SetAccumulatedImpulse(float val)
	{
		m_AcumulatedImpulse = val;
	}

	float CalculateRelativeVelocity() const;

	void WarmStart()
	{
		ApplyImpulse(m_AcumulatedImpulse);
	}

	void SetVBias(float vBias)
	{
		m_VBias = vBias;
	}

protected:
	void OrientateDirection(const Vector3& refDir)
	{
		m_Direction = m_Direction.InDirectionOf(refDir);
	}

	void ApplyImpulse(float J) const;

	float CalculateCurrentImpulse() const
	{
		return (-CalculateRelativeVelocity() + m_VBias) / m_Denom;
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	Vector3 m_Direction;
	Vector3 m_Point;
	float m_Denom;
	float m_AcumulatedImpulse;
	float m_VBias;
};

class NormalContactConstraint : public VelocityAtPointConstraint
{
private:
	static constexpr auto Beta = 0.1f;
	static constexpr auto Slop = 0.01f;

public:
	NormalContactConstraint(const Shape& s1, const Shape& s2, const Vector3& normal, const Vector3& point, float pen);

	float ApplyNextImpulse()
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();
		SetAccumulatedImpulse(MathU::Max(prevAccImpulse + delta, 0.0f));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}

	auto GetPenetration() const
	{
		return m_Penetration;
	}

private:
	float m_Penetration;
};

class FrictionContactConstraint : public VelocityAtPointConstraint
{
public:
	FrictionContactConstraint(const Shape& s1, const Shape& s2, const Vector3& manCentre, const Vector3& dir);

	float ApplyNextImpulse(float Jn)
	{
		auto prevAccImpulse = GetAccumulatedImpulse();

		auto delta = CalculateCurrentImpulse();

		static constexpr float mu = 0.5f;
		SetAccumulatedImpulse(MathU::Clamp(prevAccImpulse + delta, -mu * Jn, mu * Jn));

		auto change = GetAccumulatedImpulse() - prevAccImpulse;
		ApplyImpulse(change);

		return change;
	}
};


class JointConstraint : public VelocityAtPointConstraint
{
public:
	JointConstraint(PhysicsObject& b1, PhysicsObject& b2) :
		VelocityAtPointConstraint(b1, b2, 0.0f) // TODO - vel bias is needed for error correction
	{
	}

	float ApplyNextImpulse()
	{
		auto delta = CalculateCurrentImpulse();
		SetAccumulatedImpulse(GetAccumulatedImpulse() + delta);
		ApplyImpulse(delta);
		return delta;
	}
};

class RotationalJointCostraint
{
private:
	float CalculateCurrrentImpulse();

	void ApplyImpulse(float imp);

	void ReCalculateDenom();

public:
	RotationalJointCostraint(PhysicsObject& b1, PhysicsObject& b2) : 
		m_B1(&b1), m_B2(&b2), m_AccImpulse(0.0f), m_IsOff(false), m_VBias(0.0f)
	{
	}

	bool IsConstrained() const
	{
		return !m_IsOff;
	}

	void ResetV(const Vector3& v)
	{
		m_V = v;
		ReCalculateDenom();
	}

	void ResetV(const Matrix4& anchorTran, const Matrix4& otherTran)
	{
		m_V = Vector3::Cross(anchorTran.Cols[m_IAnchor], otherTran.Cols[m_IOther]).Normalized();
		ReCalculateDenom();
	}

	void CalculateVBias(const Matrix4& anchorTran, const Matrix4& otherTran)
	{
		static constexpr float K = 0.01f / PhysicsTime::FixedDeltaTime;
		m_VBias = 0.0f;// K * Vector3::Dot(anchorTran.Cols[m_IAnchor], otherTran.Cols[m_IOther]);
	}

	float ApplyNextImpulse()
	{
		if (m_IsOff)
			return 0.0f;

		auto imp = CalculateCurrrentImpulse();
		m_AccImpulse += imp;
		ApplyImpulse(imp);
		return imp;
	}

	void WarmStart()
	{
		ApplyImpulse(m_AccImpulse);
	}

	void InitIndexes(int iA, int iO)
	{
		m_IAnchor = iA;
		m_IOther = iO;
	}

	void TurnOff()
	{
		m_IsOff = true;
	}

	const auto& GetV() const
	{
		return m_V;
	}

private:
	Vector3 m_V;
	float m_Denom;

	PhysicsObject* m_B1;
	PhysicsObject* m_B2;

	float m_AccImpulse;
	float m_VBias;

	int m_IAnchor;
	int m_IOther;

	bool m_IsOff;
};