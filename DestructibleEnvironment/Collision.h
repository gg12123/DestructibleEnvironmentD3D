#pragma once
#include <memory>
#include "CollisionDetector.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Rigidbody.h"
#include "StaticBody.h"

class Collision
{
private:
	void HandleCollision(PhysicsObject& body1, PhysicsObject& body2)
	{
		m_Contacts.clear();
		m_Detector.FindContacts(body1, body2, m_Contacts);

		for (auto& c : m_Contacts)
			m_Responder.CalculateResponse(c, body1, body2);
	}

public:
	void DetectAndRespond(const std::vector <std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector <std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		auto dynamicCount = dynamicBodies.size();
		auto staticCount = staticBodies.size();

		// TODO - partition

		for (auto i = 0U; i < dynamicCount; i++)
		{
			auto& bodyi = *dynamicBodies[i];

			for (auto j = i + 1; j < dynamicCount; j++)
				HandleCollision(bodyi, *dynamicBodies[j]);

			for (auto j = 0U; j < staticCount; j++)
				HandleCollision(bodyi, *staticBodies[j]);
		}
	}

private:
	CollisionDetector m_Detector;
	CollisionResponder m_Responder;
	std::vector<ContactManifold> m_Contacts;
};
