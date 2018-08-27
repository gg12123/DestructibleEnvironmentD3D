#pragma once
#include <memory>
#include "Rigidbody.h"
#include "StaticBody.h"

class PhysicsEngine;

class IGameTheadToPhysicsThreadAction
{
public:
	virtual void Apply(PhysicsEngine& engine) = 0;
};

class AddDynamicRigidbodyAction : public IGameTheadToPhysicsThreadAction
{
public:
	AddDynamicRigidbodyAction(std::unique_ptr<Rigidbody>&& toAdd) : m_BodyToAdd(std::move(toAdd))
	{
	}

	void Apply(PhysicsEngine& engine) override;

private:
	std::unique_ptr<Rigidbody> m_BodyToAdd;
};

class AddStaticRigidbodyAction : public IGameTheadToPhysicsThreadAction
{
public:
	AddStaticRigidbodyAction(std::unique_ptr<StaticBody>&& toAdd) : m_BodyToAdd(std::move(toAdd))
	{
	}

	void Apply(PhysicsEngine& engine) override;

private:
	std::unique_ptr<StaticBody> m_BodyToAdd;
};