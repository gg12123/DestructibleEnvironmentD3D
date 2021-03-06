#include "pch.h"
#include "Physics.h"
#include "PhysicsEngine.h"
#include "Shape.h"
#include "ShapeProxy.h"
#include "StaticShapeProxy.h"
#include "DynamicBodyProxy.h"
#include "InitialShapeCreator.h"
#include "World.h"
#include "StaticBody.h"
#include "ShapeDestructor.h"

void Physics::AddNewProxy(ShapeProxy& proxy, CompoundShape& physicsShape)
{
	m_ShapeProxies.push_back(&proxy);
}

StaticBody & Physics::AddStaticRigidbody(StaticShapeProxy& proxy)
{
	auto body = std::unique_ptr<StaticBody>(new StaticBody());
	m_ShapeCreator.Create(*body, proxy);

	auto& toRet = *body;
	toRet.InitMassProperties(proxy.GetTransform());

	m_Engine.AddStaticBody(std::move(body));
	AddNewProxy(proxy, toRet);

	return toRet;
}

Rigidbody & Physics::AddDynamicRigidbody(DynamicBodyProxy& proxy)
{
	auto body = std::unique_ptr<Rigidbody>(new Rigidbody());
	m_ShapeCreator.Create(*body, proxy);

	auto& b = *body;

	b.SetLinearDamping(0.25f);
	b.SetAngularDamping(0.5f);
	b.InitMassProperties(proxy.GetTransform());

	m_Engine.AddDynamicBody(std::move(body));
	AddNewProxy(proxy, b);

	return b;
}

void Physics::DestructBody(DynamicBodyProxy& body, const Impulse& casue)
{
	auto& toSplit = body.GetPhysicsBody();
	auto& col = body.GetColour();

	static std::vector<Rigidbody*> newBodiesFromDestruct;
	static ShapeDestructor<Rigidbody> destructor;

	newBodiesFromDestruct.clear();
	destructor.Destruct(toSplit, casue, newBodiesFromDestruct);

	for (auto newBody : newBodiesFromDestruct)
	{
		if (newBody != &toSplit)
		{
			m_Engine.AddDynamicBody(std::unique_ptr<Rigidbody>(newBody));
			CreateShapeProxyForBodyAddedByDestruct(*newBody, col);
		}

		newBody->CopyVelocity(toSplit);
		newBody->CopyDamping(toSplit);
	}
}

RayCastHit<ShapeProxy> Physics::RayCast(const Ray& r) const
{
	auto hit = m_Engine.RayCast(r);
	if (hit.Hit())
	{
		auto hitShape = hit.GetHitObject();
		return RayCastHit<ShapeProxy>(&hitShape->GetProxy(), hit.GetHitPoint());
	}
	return RayCastHit<ShapeProxy>(nullptr, Vector3::Zero());
}

void Physics::Syncronise()
{
	for (auto b : m_ShapeProxies)
		b->Syncronise();
}

void Physics::TickPhysicsEngine()
{
	m_Engine.SimulateOneTimeStep();
}

void Physics::CreateShapeProxyForBodyAddedByDestruct(Rigidbody& body, const Vector3& col)
{
	auto prox = new DynamicBodyProxy(body);
	prox->SetColour(col);

	// this proxy has been created for a shape that was added by the physics thread
	// so it needs registering with the world
	m_World->RegisterEntity(std::unique_ptr<Entity>(prox));

	AddNewProxy(*prox, body);
}