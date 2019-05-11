#pragma once
#include <array>
#include "Matrix.h"
#include "Shape.h"
#include "Constraints.h"

class PhysicsObject;

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

	void RefreshLocalTransforms();

public:
	Joint(const Matrix4& worldTransform, PhysicsObject& anchorObject, PhysicsObject& otherObject,
		const Shape& shapeAnchor, const Shape& shapeOther, const std::array<bool, 3>& rotConstrained);

	void UpdateWorldTransform();

	void Refresh();

	PhysicsObject& GetAnchorObj() const
	{
		return *m_AnchorObject;
	}

	PhysicsObject& GetOtherObj() const
	{
		return *m_OtherObject;
	}

	PhysicsObject& GetOther(const PhysicsObject& obj) const
	{
		if (&obj == m_AnchorObject)
			return *m_OtherObject;

		if (&obj == m_OtherObject)
			return *m_AnchorObject;

		assert(false);
		return *m_AnchorObject;
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

	bool IsAttachedTo(const Shape& s) const
	{
		return &s == m_AttachedShapeAnchor || &s == m_AttachedShapeOther;
	}

	const auto GetRotConstraint1() const
	{
		return m_RotConstraint1;
	}

	const auto GetRotConstraint2() const
	{
		return m_RotConstraint2;
	}

	const auto GetRotConstraint3() const
	{
		return m_RotConstraint3;
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
