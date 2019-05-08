#pragma once
#include "Matrix.h"
#include "PhysicsObject.h"
#include "Shape.h"
#include "Constraints.h"

class Joint
{
public:
	Joint(const Matrix4& worldTransform, PhysicsObject& anchorObject, PhysicsObject& otherObject,
		const Shape& shapeAnchor, const Shape& shapeOther) :
		m_AnchorObject(&anchorObject), m_OtherObject(&otherObject),
		m_AttachedShapeAnchor(&shapeAnchor), m_AttachedShapeOther(&shapeOther),
		m_ConstraintX(anchorObject, otherObject),
		m_ConstraintY(anchorObject, otherObject),
		m_ConstraintZ(anchorObject, otherObject)
	{
		m_LocalTransformAnchor = m_AnchorObject->GetTransform().GetWorldToLocalMatrix() * worldTransform;
		m_LocalTransformOther = m_OtherObject->GetTransform().GetWorldToLocalMatrix() * worldTransform;
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
};
