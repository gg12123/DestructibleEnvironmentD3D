#pragma once
#include "ContactPoints.h"
#include "CollisionResponder.h"

class SequentialImpulsesSolver
{
private:
	void ApplyImpulseAt(const ContactPoint& p)
	{
		m_Reponder.CalculateResponse(ContactPlane(p.Point, p.Normal1To2), *p.Body1, *p.Body2);
	}

public:
	void Solve(const std::vector<ContactPoint>& contactPoints)
	{
		for (auto i = 0; i < 10; i++)
		{
			for (auto& c : contactPoints)
				ApplyImpulseAt(c);
		}
	}

private:
	CollisionResponder m_Reponder;
};
