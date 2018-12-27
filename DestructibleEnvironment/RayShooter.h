#pragma once
#include "Entity.h"
#include "InputChannelWrapper.h"
#include "IOnPhysicsWorldUpdated.h"

class RayShooter : public Entity, IOnPhysicsWorldUpdated
{
public:
	void OnPhysicsWorldUpdated() override;

protected:
	void Awake() override;

private:
	InputChannelWrapper m_LeftMouseButton;
};
