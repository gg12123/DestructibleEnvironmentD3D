#pragma once
#include "Entity.h"
#include "InputChannelWrapper.h"

class RayShooter : public Entity
{
public:
	void Update() override;

protected:
	void Awake() override;

private:
	InputChannelWrapper m_LeftMouseButton;
};
