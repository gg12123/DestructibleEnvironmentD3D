#pragma once
#include "CompoundShape.h"
#include "Rigidbody.h"
#include "PoolOfRecyclables.h"
#include "Debug.h"

class Island
{
public:
	void Clear()
	{
		m_Manifolds.clear();
		m_Bodies.clear();
		m_BodiesAsleepOnAdd.clear();
		m_BodiesAwakeOnAdd.clear();
	}

	void AddBody(Rigidbody& b)
	{
		m_Bodies.emplace_back(&b);
		b.SetIsland(*this);

		if (b.IsStill())
			m_StillCount++;

		if (b.IsAwake())
			m_BodiesAwakeOnAdd.emplace_back(&b);
		else
			m_BodiesAsleepOnAdd.emplace_back(&b);
	}

	void AddManidold(int mi)
	{
		m_Manifolds.emplace_back(mi);
	}

	const auto& GetManifolds() const
	{
		return m_Manifolds;
	}

	const auto& GetBodies() const
	{
		return m_Bodies;
	}

	void OnAllBodiesAdded()
	{
		if (m_StillCount == m_Bodies.size())
		{
			for (auto b : m_BodiesAwakeOnAdd)
				b->GoToSleep();

			m_IsAwake = false;
		}
		else
		{
			for (auto b : m_BodiesAsleepOnAdd)
				b->WakeUp();

			m_IsAwake = true;
		}
	}

	bool IsAwake() const
	{
		return m_IsAwake;
	}

	bool IsSingleton() const
	{
		return m_Manifolds.size() == 0u;
	}

private:
	std::vector<int> m_Manifolds;
	std::vector<Rigidbody*> m_Bodies;
	std::vector<Rigidbody*> m_BodiesAwakeOnAdd;
	std::vector<Rigidbody*> m_BodiesAsleepOnAdd;
	bool m_IsAwake;
	int m_StillCount;
};

class Islands
{
public:
	struct IslandNode
	{
		Rigidbody* Owner;

		// Parralel vectors
		std::vector<int> LinkedNodes;
		std::vector<int> ManifoldIndexs;

		bool Visited;

		void SetStatic()
		{
			Owner = nullptr;
			Visited = false;
		}

		bool IsStaticNode() const
		{
			return !Owner;
		}

		void ClearFor(Rigidbody& c)
		{
			LinkedNodes.clear();
			ManifoldIndexs.clear();
			Owner = &c;
			Visited = false;
		}
	};

	Islands() : m_IslandPool(25, []() { return std::unique_ptr<Island>( new Island()); })
	{
	}

	void RegisterManifold(const CompoundShape& a, const CompoundShape& b)
	{
		auto maniIndex = m_ManifoldTraversed.size();
		m_ManifoldTraversed.emplace_back(false);

		auto& aNode = m_Nodes[a.GetCompoundShapeId()];
		if (a.IsStatic())
		{
			aNode.SetStatic();
		}
		else
		{
			aNode.ManifoldIndexs.emplace_back(maniIndex);
			aNode.LinkedNodes.emplace_back(b.GetCompoundShapeId());
		}

		auto& bNode = m_Nodes[b.GetCompoundShapeId()];
		if (b.IsStatic())
		{
			bNode.SetStatic();
		}
		else
		{
			bNode.ManifoldIndexs.emplace_back(maniIndex);
			bNode.LinkedNodes.emplace_back(a.GetCompoundShapeId());
		}
	}

	// Call this with total object (inculde statics) count at start
	// of each tick so the reg function can assume the nodes vector
	// is big enough
	void Resize(int count)
	{
		m_Nodes.resize(count);
	}

	void ClearCollisonData(Rigidbody& c)
	{
		m_Nodes[c.GetCompoundShapeId()].ClearFor(c);
	}

	IslandNode* FindUnvisitedNode(const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		for (auto i = m_CurrUnvisitedTip; i < dynamicBodies.size(); i++)
		{
			auto& b = *dynamicBodies[i];
			if (!b.GetIsland())
			{
				m_CurrUnvisitedTip = i + 1u;
				return &m_Nodes[b.GetCompoundShapeId()];
			}
		}
		m_CurrUnvisitedTip = dynamicBodies.size();
		return nullptr;
	}

	Island& NextIsland()
	{
		auto& i = *m_IslandPool.Recycle();
		i.Clear();
		return i;
	}

	void CreateIsland(IslandNode& root)
	{
		assert(!root.Visited);

		auto& island = NextIsland();
		m_NodeStack.push(&root);

		while (!m_NodeStack.empty())
		{
			auto node = m_NodeStack.top();
			m_NodeStack.pop();

			assert(!node->IsStaticNode());

			if (!node->Visited)
			{
				island.AddBody(*node->Owner);
				node->Visited = true;
			}

			auto& maniIndexs = node->ManifoldIndexs;
			auto& linkedNodes = node->LinkedNodes;
			
			for (auto i = 0u; i < maniIndexs.size(); i++)
			{
				auto mani = maniIndexs[i];
				if (!m_ManifoldTraversed[mani])
				{
					island.AddManidold(mani);
					m_ManifoldTraversed[mani] = true;

					auto& linkedNode = m_Nodes[linkedNodes[i]];
					if (!linkedNode.IsStaticNode())
					{
						m_NodeStack.push(&linkedNode);
					}
				}
			}
		}

		island.OnAllBodiesAdded();
		m_Islands.emplace_back(&island);
	}

	void ReCalculateIslands(const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		for (auto& b : dynamicBodies)
			b->ClearIsland();

		m_CurrUnvisitedTip = 0u;
		m_Islands.clear();
		m_IslandPool.Reset();

		auto rootNode = FindUnvisitedNode(dynamicBodies);

		while (rootNode)
		{
			CreateIsland(*rootNode);
			rootNode = FindUnvisitedNode(dynamicBodies);
		}

		m_ManifoldTraversed.clear();
	}

	const auto& GetIslands() const
	{
		return m_Islands;
	}

private:
	// The nodes are keyed by shape ID
	std::vector<IslandNode> m_Nodes;

	uint32 m_CurrUnvisitedTip;
	std::vector<Island*> m_Islands;
	std::stack<IslandNode*> m_NodeStack;
	std::vector<uint8> m_ManifoldTraversed;
	PoolOfRecyclables<std::unique_ptr<Island>> m_IslandPool;
};
