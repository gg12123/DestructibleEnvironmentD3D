#pragma once
#include <array>
#include "Matrix.h"
#include "PhysicsObject.h"
#include "Shape.h"
#include "Constraints.h"

class Joint
{
private:
	int ConstranedCount(const std::array<bool, 3>& isConstrained) const
	{
		auto conCount = 0;
		for (auto b : isConstrained)
		{
			if (b)
				conCount++;
		}
		return conCount;
	}

	int FindHingeIndex(const std::array<bool, 3>& isConstrained) const
	{
		for (auto i = 0; i < 3; i++)
		{
			if (!isConstrained[i])
				return i;
		}
		assert(false);
		return 0;
	}

	int FindConstrainedIndex(const std::array<bool, 3>& isConstrained) const
	{
		for (auto i = 0; i < 3; i++)
		{
			if (isConstrained[i])
				return i;
		}
		assert(false);
		return 0;
	}

	void InitRotConstraints(const std::array<bool, 3>& isConstrained)
	{
		auto conCount = ConstranedCount(isConstrained);
		switch (conCount)
		{
		case 0:
		{
			m_RotConstraint1.TurnOff();
			m_RotConstraint2.TurnOff();
			m_RotConstraint3.TurnOff();
			break;
		}
		case 1:
		{
			auto conIndex = FindConstrainedIndex(isConstrained);
			m_RotConstraint1.InitIndexes((conIndex + 1) % 3, (conIndex + 2) % 3);
			m_RotConstraint2.TurnOff();
			m_RotConstraint3.TurnOff();
			break;
		}
		case 2:
		{
			auto hingeIndex = FindHingeIndex(isConstrained);
			m_RotConstraint1.InitIndexes(hingeIndex, (hingeIndex + 1) % 3);
			m_RotConstraint2.InitIndexes(hingeIndex, (hingeIndex + 2) % 3);
			m_RotConstraint3.TurnOff();
			break;
		}
		case 3:
		{
			auto i = 0;
			m_RotConstraint1.InitIndexes(2, 0);
			m_RotConstraint2.InitIndexes(2, 1);
			m_RotConstraint3.InitIndexes(1, 0);
			break;
		}
		default:
			break;
		}
	}

public:
	Joint(const Matrix4& worldTransform, PhysicsObject& anchorObject, PhysicsObject& otherObject,
		const Shape& shapeAnchor, const Shape& shapeOther, const std::array<bool, 3>& rotConstrained) :
		m_AnchorObject(&anchorObject), m_OtherObject(&otherObject),
		m_AttachedShapeAnchor(&shapeAnchor), m_AttachedShapeOther(&shapeOther),
		m_ConstraintX(anchorObject, otherObject),
		m_ConstraintY(anchorObject, otherObject),
		m_ConstraintZ(anchorObject, otherObject),
		m_RotConstraint1(anchorObject, otherObject),
		m_RotConstraint2(anchorObject, otherObject),
		m_RotConstraint3(anchorObject, otherObject)
	{
		m_LocalTransformAnchor = m_AnchorObject->GetTransform().GetWorldToLocalMatrix() * worldTransform;
		m_LocalTransformOther = m_OtherObject->GetTransform().GetWorldToLocalMatrix() * worldTransform;

		InitRotConstraints(rotConstrained);
	}

	void UpdateWorldTransform()
	{
		m_WorldTransformAnchor = m_AnchorObject->GetTransform().GetLocalToWorldMatrix() * m_LocalTransformAnchor;
		m_WorldTransformOther = m_OtherObject->GetTransform().GetLocalToWorldMatrix() * m_LocalTransformOther;

		auto posFromAnchor = FromColumn(m_WorldTransformAnchor.Cols[3].Floats);
		auto posFromOther = FromColumn(m_WorldTransformOther.Cols[3].Floats);
		auto pos = (posFromAnchor + posFromOther) / 2.0f;
		auto error = posFromOther - posFromAnchor;

		static constexpr float K = 0.1f / PhysicsTime::FixedDeltaTime;

		m_ConstraintX.ResetPointAndDirection(pos, FromColumn(m_WorldTransformAnchor.Cols[0].Floats));
		m_ConstraintX.SetVBias(-K * Vector3::Dot(m_ConstraintX.GetDirection(), error));

		m_ConstraintY.ResetPointAndDirection(pos, FromColumn(m_WorldTransformAnchor.Cols[1].Floats));
		m_ConstraintY.SetVBias(-K * Vector3::Dot(m_ConstraintY.GetDirection(), error));

		m_ConstraintZ.ResetPointAndDirection(pos, FromColumn(m_WorldTransformAnchor.Cols[2].Floats));
		m_ConstraintZ.SetVBias(-K * Vector3::Dot(m_ConstraintZ.GetDirection(), error));

		m_RotConstraint1.ResetV(m_WorldTransformAnchor, m_WorldTransformOther);
		m_RotConstraint2.ResetV(m_WorldTransformAnchor, m_WorldTransformOther);
		m_RotConstraint3.ResetV(Vector3::Cross(m_RotConstraint1.GetV(), m_RotConstraint2.GetV()));
	}

	PhysicsObject& GetAnchorObj() const
	{
		return *m_AnchorObject;
	}

	PhysicsObject& GetOtherObj() const
	{
		return *m_OtherObject;
	}

	void WarmStart()
	{
		m_ConstraintX.WarmStart();
		m_ConstraintY.WarmStart();
		m_ConstraintZ.WarmStart();

		m_RotConstraint1.WarmStart();
		m_RotConstraint2.WarmStart();
		m_RotConstraint3.WarmStart();
	}

	float ApplyImpulses()
	{
		auto s = 0.0f;

		s += MathU::Abs(m_ConstraintX.ApplyNextImpulse());
		s += MathU::Abs(m_ConstraintY.ApplyNextImpulse());
		s += MathU::Abs(m_ConstraintZ.ApplyNextImpulse());

		s += MathU::Abs(m_RotConstraint1.ApplyNextImpulse());
		s += MathU::Abs(m_RotConstraint2.ApplyNextImpulse());
		s += MathU::Abs(m_RotConstraint3.ApplyNextImpulse());

		return s;
	}

private:
	static inline Vector3 FromColumn(const float* col)
	{
		return Vector3(col[0], col[1], col[2]);
	}

	Matrix4 m_WorldTransformAnchor;
	Matrix4 m_WorldTransformOther;

	Matrix4 m_LocalTransformAnchor;
	Matrix4 m_LocalTransformOther;

	const Shape* m_AttachedShapeAnchor;
	const Shape* m_AttachedShapeOther;

	PhysicsObject* m_AnchorObject;
	PhysicsObject* m_OtherObject;

	JointConstraint m_ConstraintX;
	JointConstraint m_ConstraintY;
	JointConstraint m_ConstraintZ;

	RotationalJointCostraint m_RotConstraint1;
	RotationalJointCostraint m_RotConstraint2;
	RotationalJointCostraint m_RotConstraint3;
};
