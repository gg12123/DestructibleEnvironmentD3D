#include "pch.h"
#include "DynamicGridPartition.h"
#include "Rigidbody.h"
#include "MathUtils.h"

void DynamicGridPartition::CalculateGridDimensions(const std::vector<std::unique_ptr<Rigidbody>>& bodies)
{
	m_DynamicBodiesBounds.Calculate<std::vector<std::unique_ptr<Rigidbody>>, RadiusBoundsType>(bodies);

	auto origin = Vector3(m_DynamicBodiesBounds.GetXMin(), m_DynamicBodiesBounds.GetYMin(), m_DynamicBodiesBounds.GetZMin());

	auto xSizeMin = m_DynamicBodiesBounds.GetCollectionXRange() / static_cast<float>(XGridNumSquares - 1);
	auto ySizeMin = m_DynamicBodiesBounds.GetCollectionYRange() / static_cast<float>(YGridNumSquares - 1);
	auto zSizeMin = m_DynamicBodiesBounds.GetCollectionZRange() / static_cast<float>(ZGridNumSquares - 1);

	auto c = static_cast<float>(bodies.size());
	auto xSize = MathUtils::Min(m_DynamicBodiesBounds.GetXAverageRange(), xSizeMin);
	auto ySize = MathUtils::Min(m_DynamicBodiesBounds.GetYAverageRange() / c, ySizeMin);
	auto zSize = MathUtils::Min(m_DynamicBodiesBounds.GetZAverageRange() / c, zSizeMin);

	m_Grid.SetSqaureDimensions(xSize, ySize, zSize, origin);
}

void DynamicGridPartition::HandleCollisions(const std::vector<std::unique_ptr<Rigidbody>>& bodies)
{
	CalculateGridDimensions(bodies);

	m_GridSquares->Reset();
	m_Responder.Reset();

	for (auto it = bodies.begin(); it != bodies.end(); it++)
		HandleBody(**it);
}

void DynamicGridPartition::HandleBody(Rigidbody& body)
{
	body.SetLastCheckedAgainst(nullptr);

	m_Grid.GetRegionCovered(body.GetWorldBounds(), m_Region);

	for (auto i = m_Region.XStart; i < m_Region.XEnd; i++)
	{
		for (auto j = m_Region.YStart; j < m_Region.YEnd; j++)
		{
			for (auto k = m_Region.ZStart; k < m_Region.ZEnd; k++)
			{
				auto& square = m_Grid.At(i, j, k);

				if (!square->WasPlacedOnGridThisTick(m_GridSquares->NumRecycled()))
				{
					square = m_GridSquares->Recycle().get();
					square->Reset();
				}

				square->AddAndDetect(body);
			}
		}
	}
}