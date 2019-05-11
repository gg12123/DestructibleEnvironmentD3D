#include "pch.h"
#include "Joint.h"
#include "PhysicsObject.h"

Joint::Joint(const Matrix4& worldTransform, PhysicsObject& anchorObject, PhysicsObject& otherObject,
	const Shape& shapeAnchor, const Shape& shapeOther, const std::array<bool, 3>& rotConstrained) :
	m_AnchorObject(&anchorObject), m_OtherObject(&otherObject),
	m_AttachedShapeAnchor(&shapeAnchor), m_AttachedShapeOther(&shapeOther),
	m_ConstraintX(anchorObject, otherObject),
	m_ConstraintY(anchorObject, otherObject),
	m_ConstraintZ(anchorObject, otherObject),
	m_RotConstraint1(anchorObject, otherObject),
	m_RotConstraint2(anchorObject, otherObject),
	m_RotConstraint3(anchorObject, otherObject),
	m_WorldTransformAnchor(worldTransform),
	m_WorldTransformOther(worldTransform)
{
	RefreshLocalTransforms();
	InitRotConstraints(rotConstrained);
}

void Joint::UpdateWorldTransform()
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

void Joint::RefreshLocalTransforms()
{
	m_LocalTransformAnchor = m_AnchorObject->GetTransform().GetWorldToLocalMatrix() * m_WorldTransformAnchor;
	m_LocalTransformOther = m_OtherObject->GetTransform().GetWorldToLocalMatrix() * m_WorldTransformOther;
}

void Joint::Refresh()
{
	m_AnchorObject = m_AttachedShapeAnchor->GetOwner().ToPhysicsObject();
	m_OtherObject = m_AttachedShapeOther->GetOwner().ToPhysicsObject();
	RefreshLocalTransforms();
}