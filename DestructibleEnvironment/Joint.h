#pragma once
#include "Matrix.h"
#include "PhysicsObject.h"
#include "Shape.h"
#include "Constraints.h"

class Joint
{
public:
	Joint(const Matrix4& localTransform, PhysicsObject& anchorObject, PhysicsObject& otherObject,
		const Shape& shapeAnchor, const Shape& shapeOther) :
		m_LocalTransform(localTransform), m_AnchorObject(&anchorObject), m_OtherObject(&otherObject),
		m_AttachedShapeAnchor(&shapeAnchor), m_AttachedShapeOther(&shapeOther),
		m_ConstraintX(anchorObject, otherObject),
		m_ConstraintY(anchorObject, otherObject),
		m_ConstraintZ(anchorObject, otherObject)
	{
	}

	void UpdateWorldTransform()
	{
		m_WorldTransform = m_AnchorObject->GetTransform().GetLocalToWorldMatrix() * m_LocalTransform;

		auto pos = FromColumn(m_WorldTransform.Cols[3].Floats);

		m_ConstraintX.ResetPointAndDirection(pos, FromColumn(m_WorldTransform.Cols[0].Floats));
		m_ConstraintY.ResetPointAndDirection(pos, FromColumn(m_WorldTransform.Cols[1].Floats));
		m_ConstraintZ.ResetPointAndDirection(pos, FromColumn(m_WorldTransform.Cols[2].Floats));
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
	}

	float ApplyImpulses()
	{
		auto s = 0.0f;
		s += MathU::Abs(m_ConstraintX.ApplyNextImpulse());
		s += MathU::Abs(m_ConstraintY.ApplyNextImpulse());
		s += MathU::Abs(m_ConstraintZ.ApplyNextImpulse());
		return s;
	}

private:
	static inline Vector3 FromColumn(const float* col)
	{
		return Vector3(col[0], col[1], col[2]);
	}

	Matrix4 m_WorldTransform;
	Matrix4 m_LocalTransform; // local to anchor object

	const Shape* m_AttachedShapeAnchor;
	const Shape* m_AttachedShapeOther;

	PhysicsObject* m_AnchorObject;
	PhysicsObject* m_OtherObject;

	JointConstraint m_ConstraintX;
	JointConstraint m_ConstraintY;
	JointConstraint m_ConstraintZ;
};
